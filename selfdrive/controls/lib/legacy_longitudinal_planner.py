#!/usr/bin/env python3
import math

import numpy as np
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.accel_controller import AccelController
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from openpilot.selfdrive.controls.lib.legacy_longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.legacy_longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from openpilot.selfdrive.legacy_modeld.constants import T_IDXS

import cereal.messaging as messaging
from cereal import log

# MPC控制器基础参数
LON_MPC_STEP = 0.2                   # MPC预测第一步时间间隔(s)，降低以提高控制精度

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


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  """纵向运动规划器
  主要功能：
  1. 计算期望速度和加速度轨迹
  2. 处理巡航控制逻辑
  3. 实现转弯减速控制
  4. 提供前向碰撞预警(FCW)
  """
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    """初始化规划器
    参数:
    - CP: 车辆参数
    - init_v: 初始速度
    - init_a: 初始加速度
    """
    # mapd
    self.cruise_source = 'cruise'
    # 控制器初始化
    self.vision_turn_controller = VisionTurnController(CP) # 视觉转弯控制器
    self.accel_controller = AccelController() # 加速度控制器

    self.CP = CP
    self.mpc = LongitudinalMpc() # MPC控制器

    self.fcw = False
    # 状态变量初始化
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL) # 期望加速度
    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL) # 期望速度滤波器
    # 轨迹数组初始化
    self.v_desired_trajectory = np.zeros(CONTROL_N)  # 速度轨迹
    self.a_desired_trajectory = np.zeros(CONTROL_N)  # 加速度轨迹
    self.j_desired_trajectory = np.zeros(CONTROL_N)  # 加加速度轨迹
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

  def update(self, sm):
    """更新规划器状态和计算控制输出
    主要步骤：
    1. 更新参数配置
    2. 计算速度和加速度限制
    3. 执行MPC优化
    4. 生成控制轨迹
    """
    # Read params every 50 iterations
    if self.param_read_counter % 50 == 0:
      self.read_param()

    if self.param_read_counter % 300 == 0:
      self.accel_controller.set_profile(self.params.get("dp_long_accel_profile", encoding='utf-8'))
      self.vision_turn_controller.set_enabled(self.params.get_bool("dp_mapd_vision_turn_control"))
    self.param_read_counter += 1
    # 获取当前车辆状态
    v_ego = sm['carState'].vEgo # 当前车速

    v_cruise_kph = sm['controlsState'].vCruise # 巡航目标速度
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    # 确定控制状态
    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_state == LongCtrlState.off
    reset_state = reset_state or sm['carState'].gasPressed

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)
    # 状态重置逻辑
    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # Get acceleration and active solutions for custom long mpc.
    self.cruise_source, a_min_sol, v_cruise_sol = self.cruise_solutions(not reset_state, self.v_desired_filter.x,
                                                                        self.a_desired, v_cruise, sm)
    # 计算加速度限制
    accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]

    # dp - override accel using dp_long_accel_profile
    accel_limits = self.accel_controller.get_accel_limits(v_ego, accel_limits)
    # 转弯工况下的加速度限制
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05, a_min_sol)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)
    # 执行MPC优化
    self.mpc.set_weights(prev_accel_constraint, personality=self.personality)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.dp_long_use_krkeegen_tune_active = self.dp_long_use_krkeegen_tune and v_ego <= 7.5
    self.dp_long_use_df_tune_active = self.dp_long_use_df_tune and sm['radarState'].leadOne.status
    # 更新MPC控制器
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise_sol, personality=self.personality, use_df_tune=self.dp_long_use_df_tune_active, use_krkeegen_tune=self.dp_long_use_krkeegen_tune_active)
    # 生成控制轨迹
    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

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

    longitudinalPlanExt.dpE2EIsBlended = False

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
