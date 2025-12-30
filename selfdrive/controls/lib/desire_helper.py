from cereal import log
# from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.lib.drive_helpers import get_road_edge

LaneChangeState = log.LateralPlan.LaneChangeState
# 包含四个状态：
# - off: 关闭状态
# - preLaneChange: 换道准备状态
# - laneChangeStarting: 换道开始状态
# - laneChangeFinishing: 换道完成状态
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none
    # dp
    self.param_s = Params()
    # 在初始化时获取参数，避免每帧读取
    self._dp_lateral_road_edge_detected = self.param_s.get_bool("dp_lateral_road_edge_detected")
    speed_str = self.param_s.get("dp_lat_lane_change_assist_speed", encoding="utf8")
    self._dp_lat_lane_change_assist_speed = int(speed_str) * CV.KPH_TO_MS if speed_str else 0
    self._dp_lat_lane_change_abort_check = self.param_s.get_bool("dp_lat_lane_change_abort_check")

  def update(self, carstate, lateral_active, lane_change_prob,model_data=None):

    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < self._dp_lat_lane_change_assist_speed if self._dp_lat_lane_change_assist_speed > 0 else False

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # 1. 检测到转向灯，进入 preLaneChange 状态
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # 初始化 road_edge_detected 变量为 False
        road_edge_detected = False
        # dp road detected
        if self._dp_lateral_road_edge_detected and model_data is not None:
          road_edge_detected = get_road_edge(carstate, model_data, self._dp_lateral_road_edge_detected)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected and not road_edge_detected: #dp road detected
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # 2. 驾驶员确认后进入 laneChangeStarting 状态
      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # 安全检查：反向转向、反向转向灯
        # 根据参数决定是否进行安全检查
        abort_lane_change = False
        if self._dp_lat_lane_change_abort_check:
          abort_lane_change = (
            (carstate.steeringPressed and
            ((carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.left) or
              (carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.right))) or
            (carstate.leftBlinker and self.lane_change_direction == LaneChangeDirection.right) or
            (carstate.rightBlinker and self.lane_change_direction == LaneChangeDirection.left)
          )
        if abort_lane_change:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        else:
          # 原有的换道逻辑
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing

      # 3. 换道完成后进入 laneChangeFinishing 状态
      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # 安全检查：反向转向、反向转向灯
        # 根据参数决定是否进行安全检查
        abort_lane_change = False
        if self._dp_lat_lane_change_abort_check:
          abort_lane_change = (
            (carstate.steeringPressed and
            ((carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.left) or
              (carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.right))) or
            (carstate.leftBlinker and self.lane_change_direction == LaneChangeDirection.right) or
            (carstate.rightBlinker and self.lane_change_direction == LaneChangeDirection.left)
          )
        if abort_lane_change:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        else:
          # 原有的换道完成逻辑
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if one_blinker:
              self.lane_change_state = LaneChangeState.preLaneChange
            else:
              self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      
