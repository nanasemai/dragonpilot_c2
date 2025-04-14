import time
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log
from openpilot.selfdrive.hardware import EON

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
from openpilot.common.conversions import Conversions as CV

TRAJECTORY_SIZE = 33
# 根据设备类型设置相机偏移值（单位：米）
# EON是旧款设备，需要不同的偏移配置
if EON:
  CAMERA_OFFSET = -0.06  # EON设备相机偏移量为-6厘米(向左)
else:
  CAMERA_OFFSET = 0.04   # C2/C3设备相机偏移量为4厘米(向右)


PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.05
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 800.0


class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()

    self.params = Params()
    self._dp_lat_lane_priority_mode = self.params.get_bool("dp_lat_lane_priority_mode")
    self._dp_lat_lane_priority_mode_active = False
    self._dp_lat_lane_priority_mode_active_prev = False
    self.LP = LanePlanner()
    # dp // mapd - for vision turn controller
    self._d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self._dp_lat_lane_priority_mode_speed_based = int(self.params.get("dp_lat_lane_priority_mode_speed_based", encoding="utf-8")) if self._dp_lat_lane_priority_mode else 0
    self.param_read_counter = 0
    self._dp_lat_lane_change_assist_speed = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) * CV.MPH_TO_MS

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.debug_mode = debug

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=None):
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    # clip speed , lateral planning is not possible at 0 speed
    self.v_ego = max(MIN_SPEED, sm['carState'].vEgo)
    measured_curvature = sm['controlsState'].curvature

    # 确保在调用set_weights前设置当前车速
    self.lat_mpc.current_v_ego = self.v_ego

    # 同时更新p参数，确保求解器内部状态一致
    lateral_factor = max(0, self.factor1 - (self.factor2 * self.v_ego**2))
    p = np.array([self.v_ego, lateral_factor])
    # 预先设置参数，避免在set_weights中获取失败
    for i in range(LAT_MPC_N + 1):
      self.lat_mpc.solver.set(i, "p", p)

    if self.param_read_counter % 50 == 0:
      self._dp_lat_lane_priority_mode = self.params.get_bool("dp_lat_lane_priority_mode")
      if self._dp_lat_lane_priority_mode:
        self._dp_lat_lane_priority_mode_speed_based = int(self.params.get("dp_lat_lane_priority_mode_speed_based", encoding="utf-8"))
    self.param_read_counter += 1

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)

    # Lane change logic
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

    if self._dp_lat_lane_priority_mode:
      self.LP.parse_model(md)
      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    else:
      lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob

    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, model_data=md)

    if self._dp_lat_lane_priority_mode:
      d_path_xyz = self._get_laneless_laneline_d_path_xyz()
    else:
      d_path_xyz = self.path_xyz
    self._d_path_w_lines_xyz = d_path_xyz

    self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                             LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                             STEERING_RATE_COST)

    y_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
    heading_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    yaw_rate_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw_rate)
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * self.v_ego**2))
    p = np.array([self.v_ego, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    # init state for next iteration
    # mpc.u_sol is the desired second derivative of psi given x0 curv state.
    # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
    # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = time.monotonic()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 1e6 or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlan.solverCost = self.lat_mpc.cost
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.lat_mpc.x_sol.tolist()
      lateralPlan.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self._dp_lat_lane_priority_mode and self._dp_lat_lane_priority_mode_active
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlan', plan_send)

    # dp - extension
    plan_ext_send = messaging.new_message('lateralPlanExt')

    lateralPlanExt = plan_ext_send.lateralPlanExt
    lateralPlanExt.dPathWLinesX = [float(x) for x in self._d_path_w_lines_xyz[:, 0]]
    lateralPlanExt.dPathWLinesY = [float(y) for y in self._d_path_w_lines_xyz[:, 1]]

    pm.send('lateralPlanExt', plan_ext_send)

  def _get_laneless_laneline_d_path_xyz(self):
    if self._dp_lat_lane_priority_mode and self.LP is not None:
      # Turn off lanes during lane change
      if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
        self.LP.lll_prob *= self.DH.lane_change_ll_prob
        self.LP.rll_prob *= self.DH.lane_change_ll_prob

      # decide what mode should we use
      if (self.LP.lll_prob + self.LP.rll_prob)/2 < 0.3:
        self._dp_lat_lane_priority_mode_active = False
      if (self.LP.lll_prob + self.LP.rll_prob)/2 > 0.5:
        self._dp_lat_lane_priority_mode_active = True

      # when drive speed is below set speed, we set it to False
      if self._dp_lat_lane_priority_mode_active and self._dp_lat_lane_priority_mode_speed_based > 0 and self.v_ego * 3.6 < self._dp_lat_lane_priority_mode_speed_based:
        self._dp_lat_lane_priority_mode_active = False

      # perform reset mpc
      if self._dp_lat_lane_priority_mode_active != self._dp_lat_lane_priority_mode_active_prev:
        self.reset_mpc()
      self._dp_lat_lane_priority_mode_active_prev = self._dp_lat_lane_priority_mode_active

      # use default path if not active
      if not self._dp_lat_lane_priority_mode_active:
        return self.path_xyz

      # use lane planner path
      return self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
    else:
      return self.path_xyz
