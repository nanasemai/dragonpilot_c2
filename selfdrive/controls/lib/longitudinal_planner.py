#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.hybrid_modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from openpilot.selfdrive.controls.lib.accel_controller import AccelController
from openpilot.selfdrive.controls.lib.dynamic_endtoend_controller import DynamicEndtoEndController

# MPC控制器基础参数
LON_MPC_STEP = 0.15                   # MPC预测第一步时间间隔(s)，降低以提高控制精度

# 驾驶员注意力检测相关
AWARENESS_DECEL = -0.2                # 注意力分散时的减速度(m/s²)，保持平缓以避免突然减速

# 巡航控制加速度限制
A_CRUISE_MIN = -1.5                   # 最大允许减速度(m/s²)，提高紧急制动能力
A_CRUISE_MAX_VALS = [1.5, 1.2, 0.8, 0.5]  # 不同速度下的最大加速度(m/s²):
                                          # 0-20 m/s: 1.5 m/s² (强力加速)
                                          # 20-30 m/s: 1.2 m/s² (中等加速)
                                          # 30-45 m/s: 0.8 m/s² (平缓加速)
                                          # >45 m/s: 0.5 m/s² (高速巡航)
A_CRUISE_MAX_BP = [0., 20., 30., 45.]     # 加速度切换点速度阈值(m/s)

# 转弯工况下的动态控制参数
_A_TOTAL_MAX_V = [1.5, 2.0, 2.5]     # 不同速度下的最大合成加速度(m/s²):
                                     # 低速(0-15m/s): 1.5 m/s²
                                     # 中速(15-30m/s): 2.0 m/s²
                                     # 高速(>30m/s): 2.5 m/s²
_A_TOTAL_MAX_BP = [15., 30., 45.]    # 转弯工况加速度切换点速度阈值(m/s)

V_ACC_MIN = 9.72  # 35 kph

def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    # mapd
    self.cruise_source = 'cruise'
    self.vision_turn_controller = VisionTurnController(CP)
    self.accel_controller = AccelController()
    self.dynamic_endtoend_controller = DynamicEndtoEndController()

    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False
    self.dt = dt

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.param_read_counter = 0
    self.read_param()
    self.personality = log.LongitudinalPersonality.standard
    self.dp_long_use_df_tune = False
    self.dp_long_use_df_tune_active = False
    self.dp_long_use_krkeegen_tune = False
    self.dp_long_use_krkeegen_tune_active = False

  def read_param(self):
    try:
      self.personality = int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

    self.dp_long_use_df_tune = self.params.get_bool('dp_long_use_df_tune')
    self.dp_long_use_krkeegen_tune = self.params.get_bool('dp_long_use_krkeegen_tune')

  @staticmethod
  def parse_model(model_msg, model_error, v_ego, taco=False):
    if (len(model_msg.position.x) == 33 and
      len(model_msg.velocity.x) == 33 and
      len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    # rick - taco tune
    if taco:
      max_lat_accel = interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)
    return x, v, a, j

  def update(self, sm):
    if self.param_read_counter % 50 == 0:
      self.read_param()

    if self.param_read_counter % 100 == 0:
      self.accel_controller.set_profile(self.params.get("dp_long_accel_profile", encoding='utf-8'))
      self.vision_turn_controller.set_enabled(self.params.get_bool("dp_mapd_vision_turn_control"))
      self.dynamic_endtoend_controller.set_enabled(self.params.get_bool("dp_long_de2e"))

    self.param_read_counter += 1
    if self.dynamic_endtoend_controller.is_enabled():
      self.dynamic_endtoend_controller.set_mpc_fcw_crash_cnt(self.mpc.crash_cnt)
      self.mpc.mode = self.dynamic_endtoend_controller.get_mpc_mode(self.CP.radarUnavailable, sm['carState'], sm['radarState'].leadOne, sm['modelV2'], sm['controlsState'])
    else:
      self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    # dp - override accel using dp_long_accel_profile
    accel_limits = self.accel_controller.get_accel_limits(v_ego, accel_limits)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    # Get acceleration and active solutions for custom long mpc.
    self.cruise_source, a_min_sol, v_cruise_sol = self.cruise_solutions(not reset_state, self.v_desired_filter.x,
                                                                        self.a_desired, v_cruise, sm)
    if force_slow_decel:
      v_cruise_sol = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05, a_min_sol)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=self.personality)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error, v_ego, taco=True)
    self.dp_long_use_krkeegen_tune_active = self.dp_long_use_krkeegen_tune and v_ego <= V_ACC_MIN
    self.dp_long_use_df_tune_active = self.dp_long_use_df_tune and sm['radarState'].leadOne.status
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise_sol, x, v, a, j, personality=self.personality, use_df_tune=self.dp_long_use_df_tune_active, use_krkeegen_tune=self.dp_long_use_krkeegen_tune_active)

    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(self.dt, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time
    longitudinalPlan.personality = self.personality

    pm.send('longitudinalPlan', plan_send)

    # dp - extension
    plan_ext_send = messaging.new_message('longitudinalPlanExt')

    longitudinalPlanExt = plan_ext_send.longitudinalPlanExt

    longitudinalPlanExt.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlanExt.visionTurnSpeed = float(self.vision_turn_controller.v_turn)

    longitudinalPlanExt.dpE2EIsBlended = self.mpc.mode == 'blended'

    longitudinalPlanExt.longitudinalPlanExtSource = self.mpc.source if self.mpc.source != 'cruise' else self.cruise_source
    pm.send('longitudinalPlanExt', plan_ext_send)

  # mapd
  def cruise_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):
    # Update controllers
    self.vision_turn_controller.update(enabled, v_ego, a_ego, v_cruise, sm)

    # Pick solution with lowest velocity target.
    a_solutions = {'cruise': float("inf")}
    v_solutions = {'cruise': v_cruise}

    if self.vision_turn_controller.is_active:
      a_solutions['turn'] = self.vision_turn_controller.a_target
      v_solutions['turn'] = self.vision_turn_controller.v_turn

    source = min(v_solutions, key=v_solutions.get)

    return source, a_solutions[source], v_solutions[source]
