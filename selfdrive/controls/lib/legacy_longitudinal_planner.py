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
from cereal import log,car
from openpilot.selfdrive.controls.lib.acm import ACM
from openpilot.selfdrive.controls.lib.events import Events

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

EventName = car.CarEvent.EventName

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


def get_accel_from_plan(speeds, accels, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == CONTROL_N:
    v_now = speeds[0]
    a_now = accels[0]

    v_target = np.interp(action_t, T_IDXS[:CONTROL_N], speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    # 计算1秒后的目标速度，用于判断是否需要停车
    v_target_1sec = np.interp(action_t + 1.0, T_IDXS[:CONTROL_N], speeds) if len(speeds) == CONTROL_N else 0.0
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop


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
    self.events = Events()  # 初始化事件管理器
    self.acm = ACM()
    self.acm_enabled = False
    self.acm_param = False
    self.acm_downhill_param = False
    self.prev_accel_clip = [A_CRUISE_MIN, get_max_accel(0.0)]
    self.output_should_stop = False
    self.output_a_target = 0.0

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
    # 停车判定
    self.lead_start_alert = False  # 前车起步提醒标志
    self.lead_stopped_time = 0     # 前车停止时间
    self.lead_started = False      # 前车起步状态
    self.lead_start_threshold = 0.3  # 默认0.3 m/s
    self.lead_stop_threshold = 3.0   # 默认3.0秒
    self.desired_follow_distance = float('nan')  # 初始化为NaN表示无效值

  def read_param(self):
    try:
      self.personality = int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard
    self.dp_long_use_df_tune = self.params.get_bool('dp_long_use_df_tune')
    self.dp_long_use_krkeegen_tune = self.params.get_bool('dp_long_use_krkeegen_tune')
    # 读取ACM参数
    self.acm_param = self.params.get_bool('dp_lon_acm')
    self.acm_downhill_param = self.params.get_bool('dp_lon_acm_downhill')
    # 读取前车起步提醒参数
    self.lead_start_enabled = self.params.get_bool('dp_lead_start_alert')
    self.lead_start_threshold = float(self.params.get('dp_lead_start_alert_threshold', encoding='utf8') or "3") * 0.1
    self.lead_stop_threshold = float(self.params.get('dp_lead_stop_time_threshold', encoding='utf8') or "30") * 0.1

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
    # 使用参数控制ACM状态，而不是dp_flags
    if not self.acm_enabled and self.acm_param:
      self.acm_enabled = True
      self.acm.set_enabled(True)
      if self.acm_downhill_param:
        self.acm.set_downhill_only(True)
    elif self.acm_enabled and not self.acm_param:
      self.acm_enabled = False
      self.acm.set_enabled(False)
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

    # 确保使用正确的变量名
    user_control = long_control_state == LongCtrlState.off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled
    self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)

    if self.acm.just_disabled:
      reset_state = True
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
    # 更新期望跟车距离
    self.desired_follow_distance = float(round(self.mpc.target_obstacle_distance, 1))
    # 生成控制轨迹
    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # Apply ACM post-processing to the acceleration trajectory if active
    # 只在ACM开关打开时更新期望加速度轨迹
    if self.acm_enabled:
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)
    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = max(0.0, min(self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0, V_CRUISE_MAX * CV.KPH_TO_MS))

    # 计算最终输出加速度并应用ACM处理
    # 获取纵向执行器延迟时间，取上下限的平均值
    lower = getattr(self.CP, 'longitudinalActuatorDelayLowerBound', 0.15)  # 默认下限0.15秒
    upper = getattr(self.CP, 'longitudinalActuatorDelayUpperBound', 0.15)  # 默认上限0.15秒
    action_t = ((lower + upper) / 2.0) + DT_MDL  # 计算平均延迟加上模型时间步长
    output_a_target, self.output_should_stop = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory,
                                                                  action_t=action_t, vEgoStopping=self.CP.vEgoStopping)

    # 只在ACM开关打开时执行ACM相关操作
    if self.acm_enabled and self.acm.active:
      # ACM处理和限制优化
      output_a_target = self.acm.update_output_a_target(output_a_target)

    # 应用平滑限制
    accel_clip = accel_limits_turns.copy()
    # 如果ACM激活，使用ACM的输出值和限制
    if self.acm_enabled and self.acm.active:
      self.output_a_target = output_a_target
      self.prev_accel_clip = accel_clip
    else:
      # 正常的平滑限制处理
      for idx in range(2):
          accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
      self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
      self.prev_accel_clip = accel_clip

    # 改进的前车状态检测逻辑
    if self.lead_start_enabled and sm['radarState'].leadOne.status:
      lead = sm['radarState'].leadOne
      current_time = sm.logMonoTime['radarState'] / 1e9
      v_ego = sm['carState'].vEgo
      
      # 综合判定前车停止状态 - 简化条件，提高可靠性
      is_stopped = (lead.vLead < 0.3 and lead.dRel < 30.0)
      
      if is_stopped:
        # 更新停止时间戳
        self.lead_stopped_time = current_time
        self.lead_started = False
        self.lead_start_alert = False
      # 检测前车起步状态
      elif not self.lead_started and lead.vLead > self.lead_start_threshold and \
           (current_time - self.lead_stopped_time) > self.lead_stop_threshold and \
           lead.dRel < 30.0:  # 确保前车仍在范围内
        
        self.lead_started = True
        self.lead_start_alert = True
        
        # 确保事件名称存在并触发
        if hasattr(EventName, 'leadStartAlert'):
          self.events.add(EventName.leadStartAlert)
          # 额外的日志记录用于调试
          cloudlog.info(f"Lead start alert triggered: lead speed={lead.vLead} m/s, lead distance={lead.dRel} m")
        else:
          # 如果事件未定义，创建自定义提醒
          cloudlog.warning("Lead start alert event not defined, but conditions met")
      else:
        self.lead_start_alert = False
    else:
      # 功能未启用或未检测到前车，重置状态
      self.lead_started = False
      self.lead_start_alert = False

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

    # 添加 ACM 状态
    longitudinalPlanExt.acmEnabled = self.acm_enabled
    longitudinalPlanExt.acmDownhillOnly = self.acm_downhill_param
    longitudinalPlanExt.acmActive = self.acm.active
    # 添加期望跟车距离
    longitudinalPlanExt.desiredFollowDistance = self.desired_follow_distance

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
      # 记录弯道减速参数信息，不影响原有逻辑
      cloudlog.info(f"弯道减速激活: state={self.vision_turn_controller.state}, v_turn={self.vision_turn_controller.v_turn:.2f}, a_target={self.vision_turn_controller.a_target:.2f}, v_ego={v_ego:.2f}, v_cruise={v_cruise:.2f}")

    source = min(v_solutions, key=v_solutions.get)

    return source, a_solutions[source], v_solutions[source]
