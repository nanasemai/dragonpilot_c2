import numpy as np
import math
from cereal import custom
from openpilot.common.numpy_fast import interp
from common.params import Params  # 添加导入
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
import cereal.messaging as messaging

TRAJECTORY_SIZE = 33
_MIN_V = 5.6  # Do not operate under 20km/h

# 原始常量保留，但会被参数覆盖 - 优化后的默认值更加灵敏
_ENTERING_PRED_LAT_ACC_TH = 1.0  # Predicted Lat Acc threshold to trigger entering turn state. (降低阈值提高灵敏度)
_ABORT_ENTERING_PRED_LAT_ACC_TH = 0.8  # Predicted Lat Acc threshold to abort entering state if speed drops. (降低阈值)

_TURNING_LAT_ACC_TH = 1.0  # Lat Acc threshold to trigger turning turn state. (降低阈值)
_LEAVING_LAT_ACC_TH = 0.9  # Lat Acc threshold to trigger leaving turn state. (降低阈值)
_FINISH_LAT_ACC_TH = 0.8  # Lat Acc threshold to trigger end of turn cycle. (降低阈值)

_EVAL_STEP = 4.  # mts. Resolution of the curvature evaluation. (提高分辨率)
_EVAL_START = 15.  # mts. Distance ahead where to start evaluating vision curvature. (提前开始评估)
_EVAL_LENGHT = 180.  # mts. Distance ahead where to stop evaluating vision curvature. (延长评估距离)
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)

_A_LAT_REG_MAX = 1.8  # Maximum lateral acceleration (降低最大横向加速度限制，提高安全性)

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.3, -1.5]  # min decel value allowed on ENTERING state (增强减速强度)
_ENTERING_SMOOTH_DECEL_BP = [1.0, 2.5]  # absolute value of lat acc ahead (调整断点适应新阈值)

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.3, -0.2, -0.6]  # acc value (增强减速，降低加速)
_TURNING_ACC_BP = [1.0, 1.8, 2.5]  # absolute value of current lat acc (调整断点)

_LEAVING_ACC = 0.5  # Confortble acceleration to regain speed while leaving a turn.

_MIN_LANE_PROB = 0.5  # Minimum lanes probability to allow curvature prediction based on lanes.

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


class VisionTurnController():
  def __init__(self, CP):
    self._params = Params()  # 实例化参数
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
    
    # 初始化自定义参数
    self.turn_sensitivity = 1.0  # 弯道检测灵敏度系数
    self.decel_ratio = 1.0  # 减速强度系数
    self.turn_speed_ratio = 1.0  # 弯道速度系数
    
    # 应用参数调整后的实际值
    self.entering_lat_acc_th = _ENTERING_PRED_LAT_ACC_TH
    self.abort_entering_lat_acc_th = _ABORT_ENTERING_PRED_LAT_ACC_TH
    self.entering_smooth_decel_v = _ENTERING_SMOOTH_DECEL_V.copy()
    self.turning_acc_v = _TURNING_ACC_V.copy()
    
    # 读取参数
    self._update_params()
    
    self._reset()

  def _update_params(self):
    """读取并更新UI参数"""
    try:
      # 读取弯道检测灵敏度系数 (UI值的0.1倍)
      self.turn_sensitivity = self._params.get_float("dp_vt_sensitivity") / 10.0
      if self.turn_sensitivity <= 0.1:  # 防止值过小
        self.turn_sensitivity = 1.0
        
      # 读取减速强度系数 (UI值的0.1倍)
      self.decel_ratio = self._params.get_float("dp_vt_decel_ratio") / 10.0
      if self.decel_ratio <= 0.1:  # 防止值过小
        self.decel_ratio = 1.0
        
      # 读取弯道速度系数 (UI值的0.1倍)
      self.turn_speed_ratio = self._params.get_float("dp_vt_speed_ratio") / 10.0
      if self.turn_speed_ratio <= 0.1:  # 防止值过小
        self.turn_speed_ratio = 1.0
      
      # 应用参数到实际值
      # 灵敏度调整 - 值越小越灵敏
      self.entering_lat_acc_th = _ENTERING_PRED_LAT_ACC_TH / self.turn_sensitivity
      self.abort_entering_lat_acc_th = _ABORT_ENTERING_PRED_LAT_ACC_TH / self.turn_sensitivity
      
      # 减速强度调整 - 值越大减速越强
      self.entering_smooth_decel_v = [v * self.decel_ratio for v in _ENTERING_SMOOTH_DECEL_V]
      self.turning_acc_v = [v * self.decel_ratio if v < 0 else v for v in _TURNING_ACC_V]
      
    except Exception:
      # 出错时使用默认值
      self.turn_sensitivity = 1.0
      self.decel_ratio = 1.0
      self.turn_speed_ratio = 1.0
      self.entering_lat_acc_th = _ENTERING_PRED_LAT_ACC_TH
      self.abort_entering_lat_acc_th = _ABORT_ENTERING_PRED_LAT_ACC_TH
      self.entering_smooth_decel_v = _ENTERING_SMOOTH_DECEL_V.copy()
      self.turning_acc_v = _TURNING_ACC_V.copy()

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
    self._current_lat_acc = current_curvature * self._v_ego**2
    self._max_v_for_current_curvature = math.sqrt(_A_LAT_REG_MAX / current_curvature) if current_curvature > 0 \
      else V_CRUISE_MAX * CV.KPH_TO_MS

    pred_curvatures = eval_curvature(path_poly, _EVAL_RANGE)
    max_pred_curvature = np.amax(pred_curvatures)
    self._max_pred_lat_acc = self._v_ego**2 * max_pred_curvature

    max_curvature_for_vego = _A_LAT_REG_MAX / max(self._v_ego, 0.1)**2
    lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    self._lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0

    if self._lat_acc_overshoot_ahead:
      safe_lat_acc_limit = _A_LAT_REG_MAX * 0.85
      self._v_overshoot = min(math.sqrt(safe_lat_acc_limit / max_pred_curvature), self._v_cruise_setpoint)
      self._v_overshoot_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
      
      # 更精确的速度分层和更平滑的减速过渡
      if self._v_cruise_setpoint > 33.3:  # 120km/h (高速公路)
        speed_reduction_factor = 0.8
      elif self._v_cruise_setpoint > 25.0:  # 90km/h (城市快速路)
        speed_reduction_factor = 0.85
      elif self._v_cruise_setpoint > 20.8:  # 75km/h (城市主干道)
        speed_reduction_factor = 0.88
      elif self._v_cruise_setpoint > 16.7:  # 60km/h
        speed_reduction_factor = 0.9
      elif self._v_cruise_setpoint > 13.9:  # 50km/h
        speed_reduction_factor = 0.93
      else:  # 低于50km/h
        speed_reduction_factor = 0.95
      
      self._v_overshoot = min(self._v_overshoot * speed_reduction_factor, self._v_cruise_setpoint)
      _debug(f'TVC: High LatAcc. Dist: {self._v_overshoot_distance:.2f}, v: {self._v_overshoot * CV.MS_TO_KPH:.2f}')

  def _state_transition(self):
    # In any case, if system is disabled or the feature is disabeld or gas is pressed, disable.
    if not self._op_enabled or not self._is_enabled or self._gas_pressed:
      self.state = VisionTurnControllerState.disabled
      return

    # DISABLED
    if self.state == VisionTurnControllerState.disabled:
      # Do not enter a turn control cycle if speed is low.
      if self._v_ego <= _MIN_V:
        pass
      # 使用调整后的阈值
      elif self._max_pred_lat_acc >= self.entering_lat_acc_th:
        self.state = VisionTurnControllerState.entering
    # ENTERING
    elif self.state == VisionTurnControllerState.entering:
      # Transition to Turning if current lateral acceleration is over the threshold.
      if self._current_lat_acc >= _TURNING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.turning
      # 使用调整后的阈值
      elif self._max_pred_lat_acc < self.abort_entering_lat_acc_th:
        self.state = VisionTurnControllerState.disabled
    # TURNING
    elif self.state == VisionTurnControllerState.turning:
      # Transition to Leaving if current lateral acceleration drops drops below threshold.
      if self._current_lat_acc <= _LEAVING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.leaving
    # LEAVING
    elif self.state == VisionTurnControllerState.leaving:
      # Transition back to Turning if current lateral acceleration goes back over the threshold.
      if self._current_lat_acc >= _TURNING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.turning
      # Finish if current lateral acceleration goes below threshold.
      elif self._current_lat_acc < _FINISH_LAT_ACC_TH:
        self.state = VisionTurnControllerState.disabled

  def _update_solution(self):
    # DISABLED
    if self.state == VisionTurnControllerState.disabled:
      a_target = self._a_ego
    # ENTERING
    elif self.state == VisionTurnControllerState.entering:
      # 使用调整后的减速值
      a_target = interp(self._max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, self.entering_smooth_decel_v)
      if self._lat_acc_overshoot_ahead:
        # 应用弯道速度系数调整目标速度
        v_overshoot_adjusted = self._v_overshoot * self.turn_speed_ratio
        a_target = min((v_overshoot_adjusted**2 - self._v_ego**2) / (2 * self._v_overshoot_distance), a_target)
      _debug(f'TVC Entering: Overshooting: {self._lat_acc_overshoot_ahead}')
      _debug(f'    Decel: {a_target:.2f}, target v: {self.v_turn * CV.MS_TO_KPH}')
    # TURNING
    elif self.state == VisionTurnControllerState.turning:
      # 使用调整后的加速度值
      a_target = interp(self._current_lat_acc, _TURNING_ACC_BP, self.turning_acc_v)
    # LEAVING
    elif self.state == VisionTurnControllerState.leaving:
      a_target = _LEAVING_ACC

    # update solution values.
    self._a_target = a_target

  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, sm):
    self._op_enabled = enabled
    self._gas_pressed = sm['carState'].gasPressed
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._v_cruise_setpoint = v_cruise_setpoint

    # 定期更新参数
    self._update_params()
    self._update_calculations(sm)
    self._state_transition()
    self._update_solution()

  def set_enabled(self, enabled):
    self._is_enabled = enabled
