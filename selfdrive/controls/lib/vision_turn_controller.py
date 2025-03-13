import numpy as np
import math
from numpy import clip
from cereal import custom
from openpilot.common.numpy_fast import interp
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
import cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
# 轨迹预测参数
TRAJECTORY_SIZE = 33                  # 增加预测点数量，提高预测精度
_MIN_V = 5.0                         # 降低最低运行速度到18km/h，提升低速工况适应性

# 转向状态触发阈值 (单位: m/s²)
_ENTERING_PRED_LAT_ACC_TH = 1.2      # 降低进入阈值，提前介入转向控制
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.0 # 相应调整取消阈值，保持合适的触发差值

_TURNING_LAT_ACC_TH = 1.3            # 降低转向阈值，使转向更平顺

_LEAVING_LAT_ACC_TH = 1.3            # 调整离开阈值，与进入阈值对应
_FINISH_LAT_ACC_TH = 1.1             # 调整结束阈值，确保平顺退出

# 视觉曲率评估参数 (单位: 米)
_EVAL_STEP = 5.                      # 减小评估步长，提高采样精度
_EVAL_START = 20.                    # 缩短起始距离，提前规划
_EVAL_LENGHT = 150.                  # 增加评估距离，提高预见性
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)

# 横向加速度限制 (单位: m/s²)
_A_LAT_REG_MAX = 2                 # 降低最大横向加速度，提高舒适性

# 速度期望时间范围 (单位: 秒)
_NO_OVERSHOOT_TIME_HORIZON = 3.5     # 缩短预测时间，提高控制响应性

# 进入转向时的减速控制参数 (单位: m/s²)
_ENTERING_SMOOTH_DECEL_V = [-0.15, -0.8]  # 优化减速度范围，使减速更平顺
_ENTERING_SMOOTH_DECEL_BP = [1.2, 2.8]    # 调整触发点，与新的加速度阈值匹配

# 转向过程中的加速度控制参数 (单位: m/s²)
_TURNING_ACC_V = [0.4, 0., -0.3]     # 降低加减速幅度，提升舒适性
_TURNING_ACC_BP = [1.4, 2.1, 2.8]    # 优化分段点，使加减速更平滑

# 退出转向时的加速度 (单位: m/s²)
_LEAVING_ACC = 0.4                    # 降低退出加速度，确保平顺性

# 车道线概率要求
_MIN_LANE_PROB = 0.65                 # 提高概率阈值，增加预测可靠性

# 动态转向控制参数
_SPEED_FACTOR = 0.8  # 速度影响因子
_CURVE_FACTOR = 1.2  # 曲率影响因子
_COMFORT_FACTOR = 0.9  # 舒适度因子

_DEBUG = False


def _debug(msg):
  if not _DEBUG:
    return
  print(msg)


VisionTurnControllerState = custom.LongitudinalPlanExt.VisionTurnControllerState


def eval_curvature(poly, x_vals):
  """
  This function returns a vector with the curvature based on path defined by `poly`
  evaluated on distance vector `x_vals`
  """
  # https://en.wikipedia.org/wiki/Curvature#  Local_expressions
  def curvature(x):
    a = abs(2 * poly[1] + 6 * poly[0] * x) / (1 + (3 * poly[0] * x**2 + 2 * poly[1] * x + poly[2])**2)**(1.5)
    return a

  return np.vectorize(curvature)(x_vals)


def eval_lat_acc(v_ego, x_curv):
  """
  This function returns a vector with the lateral acceleration based
  for the provided speed `v_ego` evaluated over curvature vector `x_curv`
  """

  def lat_acc(curv):
    a = v_ego**2 * curv
    return a

  return np.vectorize(lat_acc)(x_curv)


def _description_for_state(turn_controller_state):
  if turn_controller_state == VisionTurnControllerState.disabled:
    return 'DISABLED'
  if turn_controller_state == VisionTurnControllerState.entering:
    return 'ENTERING'
  if turn_controller_state == VisionTurnControllerState.turning:
    return 'TURNING'
  if turn_controller_state == VisionTurnControllerState.leaving:
    return 'LEAVING'


class VisionTurnController:
  def __init__(self, CP):
    # self._params = Params()
    self._CP = CP
    self._op_enabled = False
    self._gas_pressed = False
    self._is_enabled = False
    self._last_params_update = 0.
    self._v_cruise_setpoint = 0.
    self._v_ego = 0.
    self._a_ego = 0.
    self._a_target = 0.
    self._v_overshoot = 0.
    self._state = VisionTurnControllerState.disabled
    self._sm = messaging.SubMaster(['lateralPlanExt'])
    self._last_pred_lat_acc = 0.  # 记录上一次预测的横向加速度
    self._acc_history = []        # 加速度历史记录
    self._curve_history = []      # 曲率历史记录
    self._reset()


  def _get_dynamic_thresholds(self):
    """计算动态阈值"""
    # 确保 _v_ego 不小于最小速度且不大于最大速度
    v_ego_safe = clip(self._v_ego, _MIN_V, V_CRUISE_MAX * CV.KPH_TO_MS)

    # 优化速度修正因子计算
    speed_factor = clip(v_ego_safe / 20.0, 0.5, 1.0)  # 基准速度20m/s
    speed_mod = interp(speed_factor,
                      [0.5, 1.0],
                      [1.0, _SPEED_FACTOR])

    # 基于曲率变化调整阈值，添加异常值过滤
    curve_rate = 0.0
    if len(self._curve_history) > 1:
        delta = self._curve_history[-1] - self._curve_history[-2]
        # 添加更严格的异常值过滤
        if 0.0 <= abs(delta) < 1.0:  # 合理的曲率变化范围
            curve_rate = delta / DT_MDL

    curve_mod = clip(
        interp(abs(curve_rate),
               [0.0, 2.0],
               [1.0, _CURVE_FACTOR]),
        0.5, 1.5  # 限制修正系数范围
    )

    return speed_mod, curve_mod

  @property
  def state(self):
    return self._state

  @state.setter
  def state(self, value):
    if value != self._state:
      _debug(f'TVC: TurnVisionController state: {_description_for_state(value)}')
      if value == VisionTurnControllerState.disabled:
        self._reset()
    self._state = value

  @property
  def a_target(self):
    return self._a_target if self.is_active else self._a_ego

  @property
  def v_turn(self):
    if not self.is_active:
      return self._v_cruise_setpoint
    return self._v_overshoot if self._lat_acc_overshoot_ahead \
      else self._v_ego + self._a_target * _NO_OVERSHOOT_TIME_HORIZON

  @property
  def is_active(self):
    return self._state != VisionTurnControllerState.disabled

  def _reset(self):
    self._current_lat_acc = 0.
    self._max_v_for_current_curvature = 0.
    self._max_pred_lat_acc = 0.
    self._v_overshoot_distance = 200.
    self._lat_acc_overshoot_ahead = False

  def _update_calculations(self, sm):
    # Get path polynomial aproximation for curvature estimation from model data.
    path_poly = None
    model_data = sm['modelV2']

    # 1. When the probability of lanes is good enough, compute polynomial from lanes as they are way more stable
    # on current mode than drving path.
    if model_data is not None and len(model_data.laneLines) == 4 and len(model_data.laneLines[0].t) == TRAJECTORY_SIZE:
      ll_x = model_data.laneLines[1].x  # left and right ll x is the same
      lll_y = np.array(model_data.laneLines[1].y)
      rll_y = np.array(model_data.laneLines[2].y)
      l_prob = model_data.laneLineProbs[1]
      r_prob = model_data.laneLineProbs[2]
      lll_std = model_data.laneLineStds[1]
      rll_std = model_data.laneLineStds[2]

      # Reduce reliance on lanelines that are too far apart or will be in a few seconds
      width_pts = rll_y - lll_y
      prob_mods = []
      for t_check in [0.0, 1.5, 3.0]:
        width_at_t = interp(t_check * (self._v_ego + 7), ll_x, width_pts)
        prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      mod = min(prob_mods)
      l_prob *= mod
      r_prob *= mod

      # Reduce reliance on uncertain lanelines
      l_std_mod = interp(lll_std, [.15, .3], [1.0, 0.0])
      r_std_mod = interp(rll_std, [.15, .3], [1.0, 0.0])
      l_prob *= l_std_mod
      r_prob *= r_std_mod

      # Find path from lanes as the average center lane only if min probability on both lanes is above threshold.
      if l_prob > _MIN_LANE_PROB and r_prob > _MIN_LANE_PROB:
        c_y = width_pts / 2 + lll_y
        path_poly = np.polyfit(ll_x, c_y, 3)

    # 2. If not polynomial derived from lanes, then derive it from compensated driving path with lanes as
    # provided by `lateralPlanner`.
    lat_planner_data = self._sm['lateralPlanExt']
    self._sm.update(0)
    if path_poly is None and lat_planner_data is not None and len(lat_planner_data.dPathWLinesX) > 0 \
        and lat_planner_data.dPathWLinesX[0] > 0:
      path_poly = np.polyfit(lat_planner_data.dPathWLinesX, lat_planner_data.dPathWLinesY, 3)

    # 3. If no polynomial derived from lanes or driving path, then provide a straight line poly.
    if path_poly is None:
      path_poly = np.array([0., 0., 0., 0.])

    current_curvature = abs(
      sm['carState'].steeringAngleDeg * CV.DEG_TO_RAD / (self._CP.steerRatio * self._CP.wheelbase))

    # 计算当前横向加速度
    self._current_lat_acc = current_curvature * self._v_ego**2

    # 更新历史记录，限制最大长度为10
    if len(self._acc_history) >= 10:
        self._acc_history = self._acc_history[1:]
        self._curve_history = self._curve_history[1:]
    self._acc_history.append(self._current_lat_acc)
    self._curve_history.append(current_curvature)

    # 使用动态阈值
    speed_mod, curve_mod = self._get_dynamic_thresholds()
    comfort_factor = _COMFORT_FACTOR if self._current_lat_acc > 1.5 else 1.0

    # 应用动态修正系数
    self._max_v_for_current_curvature = math.sqrt(_A_LAT_REG_MAX * speed_mod * curve_mod * comfort_factor / current_curvature) if current_curvature > 0 \
      else V_CRUISE_MAX * CV.KPH_TO_MS

    pred_curvatures = eval_curvature(path_poly, _EVAL_RANGE)
    max_pred_curvature = np.amax(pred_curvatures)

    # 使用动态修正的预测横向加速度
    self._max_pred_lat_acc = (self._v_ego**2 * max_pred_curvature) * speed_mod * curve_mod * comfort_factor

    # 动态调整最大曲率阈值
    max_curvature_for_vego = (_A_LAT_REG_MAX * speed_mod * curve_mod * comfort_factor) / max(self._v_ego, 0.1)**2
    lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    self._lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0

    if self._lat_acc_overshoot_ahead:
        # 使用动态修正的超调速度
        self._v_overshoot = min(math.sqrt(_A_LAT_REG_MAX * speed_mod * curve_mod * comfort_factor / max_pred_curvature),
                               self._v_cruise_setpoint)
        self._v_overshoot_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
        _debug(f'TVC: High LatAcc. Dist: {self._v_overshoot_distance:.2f}, v: {self._v_overshoot * CV.MS_TO_KPH:.2f}')

    # 保存当前预测值用于下次计算
    self._last_pred_lat_acc = self._max_pred_lat_acc

  def _state_transition(self):
    if not self._op_enabled or not self._is_enabled or self._gas_pressed or abs(self._v_ego) < 0.1:
      self.state = VisionTurnControllerState.disabled
      return

    if not hasattr(self, '_state_time'):
      self._state_time = 0.0
    else:
      self._state_time += DT_MDL

    speed_mod, curve_mod = self._get_dynamic_thresholds()
    comfort_factor = _COMFORT_FACTOR if self._current_lat_acc > 1.5 else 1.0

    entering_th = _ENTERING_PRED_LAT_ACC_TH * speed_mod * curve_mod * comfort_factor
    turning_th = _TURNING_LAT_ACC_TH * speed_mod * curve_mod * comfort_factor
    leaving_th = _LEAVING_LAT_ACC_TH * speed_mod * curve_mod * comfort_factor
    finish_th = _FINISH_LAT_ACC_TH * speed_mod * curve_mod * comfort_factor
    MIN_STATE_TIME = 0.5

    current_state = self.state
    if current_state == VisionTurnControllerState.disabled:
      if self._max_pred_lat_acc >= entering_th and \
         abs(self._max_pred_lat_acc - self._last_pred_lat_acc) < 0.5 and \
         self._state_time >= MIN_STATE_TIME:
        self.state = VisionTurnControllerState.entering
        self._state_time = 0.0

    elif current_state == VisionTurnControllerState.entering:
      if self._current_lat_acc >= turning_th and self._state_time >= MIN_STATE_TIME:
        self.state = VisionTurnControllerState.turning
        self._state_time = 0.0
      elif self._max_pred_lat_acc < entering_th * 0.8:
        self.state = VisionTurnControllerState.disabled
        self._state_time = 0.0

    elif current_state == VisionTurnControllerState.turning:
      if self._current_lat_acc <= leaving_th and self._state_time >= MIN_STATE_TIME:
        self.state = VisionTurnControllerState.leaving
        self._state_time = 0.0

    elif current_state == VisionTurnControllerState.leaving:
      if self._current_lat_acc >= turning_th and self._state_time >= MIN_STATE_TIME:
        self.state = VisionTurnControllerState.turning
        self._state_time = 0.0
      elif self._current_lat_acc < finish_th and \
           self._max_pred_lat_acc < finish_th and \
           self._state_time >= MIN_STATE_TIME:
        self.state = VisionTurnControllerState.disabled
        self._state_time = 0.0

    if current_state != self.state:
      _debug(f"State changed from {_description_for_state(current_state)} to {_description_for_state(self.state)}")

  def _update_solution(self):
    # 获取动态阈值用于调整加速度
    speed_mod, curve_mod = self._get_dynamic_thresholds()
    comfort_factor = _COMFORT_FACTOR if self._current_lat_acc > 1.5 else 1.0

    # 初始化 a_target
    a_target = self._a_ego  # 默认值设置移到开头

    # DISABLED
    if self.state == VisionTurnControllerState.disabled:
      a_target = self._a_ego

    # ENTERING
    elif self.state == VisionTurnControllerState.entering:
      # 使用动态修正的减速度
      base_decel = interp(self._max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
      a_target = base_decel * speed_mod * curve_mod * comfort_factor

      if self._lat_acc_overshoot_ahead:
        # 优化超调速度计算
        distance_factor = clip(self._v_overshoot_distance / 50.0, 0.5, 1.5)
        v_diff = self._v_overshoot - self._v_ego
        a_target = min((v_diff * distance_factor) / max(2.0, self._v_overshoot_distance/self._v_ego), a_target)

      _debug(f'TVC Entering: Overshooting: {self._lat_acc_overshoot_ahead}')
      _debug(f'    Decel: {a_target:.2f}, target v: {self.v_turn * CV.MS_TO_KPH}')

    # TURNING
    elif self.state == VisionTurnControllerState.turning:
      # 使用动态修正的转弯加速度
      base_acc = interp(self._current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
      a_target = base_acc * speed_mod * curve_mod * comfort_factor

    # LEAVING
    elif self.state == VisionTurnControllerState.leaving:
      # 使用动态修正的离开加速度
      a_target = _LEAVING_ACC * speed_mod * curve_mod * comfort_factor

    # 简化加速度限制逻辑
    self._a_target = np.clip(a_target, -2.0, 1.0)

  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, sm):
    self._op_enabled = enabled
    self._gas_pressed = sm['carState'].gasPressed
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._v_cruise_setpoint = v_cruise_setpoint

    # self._update_params()
    self._update_calculations(sm)
    self._state_transition()
    self._update_solution()

  def set_enabled(self, enabled):
    self._is_enabled = enabled
