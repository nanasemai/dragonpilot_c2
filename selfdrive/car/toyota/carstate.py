import copy

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

SteerControlType = car.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
#  这些转向故障定义似乎在LKA（扭矩）和LTA（角度）中很常见：
#  -高转向率故障：在1帧内变为21或25，然后在2秒内变为9
#  -lka/lta msg退出：先是9秒，然后是11秒，总共2秒，然后3秒。
#  如果使用其他控制命令，则在1.5秒后直接转到3
#  -初始化：只要STEER_TORQUE_SENSOR->STEER_ANGLE_initializing为1，LTA就可以报告0，
#  对LKA来说是包罗万象的
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
#-lka/lta msg退出：3（可恢复）
#-长时间高驱动扭矩：17（永久）
PERM_STEER_FAULTS = (3, 17)

ZSS_THRESHOLD = 4.0
ZSS_THRESHOLD_COUNT = 10

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    # 在配备cp.vl[“STEER_TORQUE_SENSOR”][“STEER_ANGLE”]的汽车上
    # 信号被归零到转向角开始的位置。
    # 在收到两个转向角测量值后，需要立即应用偏移
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.prev_distance_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}

    # zss
    params = Params()
    self._dp_alka = params.get_bool('dp_alka')
    self._dp_toyota_zss = params.get_bool('dp_toyota_zss')
    self._dp_zss_compute = False
    self._dp_zss_cruise_active_last = False
    self._dp_zss_angle_offset = 0.
    self._dp_zss_threshold_count = 0

    # bsm
    self.dp_toyota_enhanced_bsm = params.get_bool('dp_toyota_enhanced_bsm')
    self._left_blindspot = False
    self._left_blindspot_d1 = 0
    self._left_blindspot_d2 = 0
    self._left_blindspot_counter = 0

    self._right_blindspot = False
    self._right_blindspot_d1 = 0
    self._right_blindspot_d2 = 0
    self._right_blindspot_counter = 0
    # BSM 超时时间常量
    self._BSM_COUNTER_MAX = 100
    self._BSM_TIMEOUT = 0.5  # 秒
    self._last_bsm_update = 0.


  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = ret.gas > 805
    else:
      # TODO: find a common gas pedal percentage signal
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    # 过滤掉异常值
    wheel_speeds = [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]
    valid_speeds = [s for s in wheel_speeds if abs(s) < 150]  # 过滤掉不合理的速度值
    ret.vEgoRaw = mean(valid_speeds) if valid_speeds else 0.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    # dp - toyota zss
    # if diff between steeringAngleDeg and ZSS is too large, disable ZSS
    if self._dp_zss_threshold_count > ZSS_THRESHOLD_COUNT:
      self._dp_toyota_zss = False
    if self._dp_toyota_zss:
      try:
        zorro_steer = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]
        if abs(zorro_steer) > 500:  # 添加合理性检查
          self._dp_zss_threshold_count = ZSS_THRESHOLD_COUNT + 1
        else:
          zorro_steer = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]
          # rick - when alka is on, we check main_on state
          acc_active = (self._dp_alka and cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0) or bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
          # only compute zss offset when acc is active
          if acc_active and not self._dp_zss_cruise_active_last:
            self._dp_zss_threshold_count = 0
            self._dp_zss_compute = True # cruise was just activated, so allow offset to be recomputed
          self._dp_zss_cruise_active_last = acc_active

          # compute zss offset
          if self._dp_zss_compute:
            if abs(ret.steeringAngleDeg) > 1e-3 and abs(zorro_steer) > 1e-3:
              self._dp_zss_compute = False
              self._dp_zss_angle_offset = zorro_steer - ret.steeringAngleDeg

          # error check
          new_steering_angle_deg = zorro_steer - self._dp_zss_angle_offset
          if abs(ret.steeringAngleDeg - new_steering_angle_deg) > ZSS_THRESHOLD:
            self._dp_zss_threshold_count += 1
          else:
            ret.steeringAngleDeg = new_steering_angle_deg
      except Exception as e:
        self._dp_toyota_zss = False
        cloudlog.exception("ZSS error")

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    if self.CP.carFingerprint != CAR.MIRAI:
      ret.engineRpm = cp.vl["ENGINE_RPM"]["RPM"]

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    if ret.cruiseState.speed != 0:
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp

    if self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
      (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # ignore standstill state in certain vehicles, since pcm allows to restart with just an acceleration request
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)
    # dp - for pcm compensation
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if not self.CP.enableDsu and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    # dp distance button
    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      # distance button is wired to the ACC module (camera or radar)
      self.prev_distance_button = self.distance_button
      if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]

    # dp - acc filter - distance button logic for sdsu not radar can filter
    #self.prev_distance_button, self.distance_button = self.acc_filter_state.get_distance_button_states(self.prev_distance_button, self.distance_button)

    # Enable blindspot debug mode once (@arne182)
    # let's keep all the commented out code for easy debug purpose for future.
    if self.dp_toyota_enhanced_bsm and self.frame > 199: #self.CP.carFingerprint == CAR.PRIUS_TSS2: #not (self.CP.carFingerprint in TSS2_CAR or self.CP.carFingerprint == CAR.CAMRY or self.CP.carFingerprint == CAR.CAMRYH):
      distance_1 = cp.vl["DEBUG"].get('BLINDSPOTD1')
      distance_2 = cp.vl["DEBUG"].get('BLINDSPOTD2')
      side = cp.vl["DEBUG"].get('BLINDSPOTSIDE')

      if distance_1 is not None and distance_2 is not None and side is not None:
        if side == 65: # Left blind spot
          if distance_1 != self._left_blindspot_d1:
            self._left_blindspot_d1 = distance_1
            self._left_blindspot_counter = 100
          if distance_2 != self._left_blindspot_d2:
            self._left_blindspot_d2 = distance_2
            self._left_blindspot_counter = 100
          if self._left_blindspot_d1 > 10 or self._left_blindspot_d2 > 10:
            self._left_blindspot = True
        elif side == 66: # Right blind spot
          if distance_1 != self._right_blindspot_d1:
            self._right_blindspot_d1 = distance_1
            self._right_blindspot_counter = 100
          if distance_2 != self._right_blindspot_d2:
            self._right_blindspot_d2 = distance_2
            self._right_blindspot_counter = 100
          if self._right_blindspot_d1 > 10 or self._right_blindspot_d2 > 10:
            self._right_blindspot = True

        if self._left_blindspot_counter > 0:
          self._left_blindspot_counter -= 1
        else:
          self._left_blindspot = False
          self._left_blindspot_d1 = 0
          self._left_blindspot_d2 = 0

        if self._right_blindspot_counter > 0:
          self._right_blindspot_counter -= 1
        else:
          self._right_blindspot = False
          self._right_blindspot_d1 = 0
          self._right_blindspot_d2 = 0

        ret.leftBlindspot = self._left_blindspot
        ret.rightBlindspot = self._right_blindspot

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("GEAR_PACKET", 1),
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.carFingerprint != CAR.MIRAI:
      messages.append(("ENGINE_RPM", 42))

    if CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      messages.append(("DSU_CRUISE", 5))
      messages.append(("PCM_CRUISE_ALT", 1))
    else:
      messages.append(("PCM_CRUISE_2", 33))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      messages.append(("GAS_SENSOR", 50))

    if CP.enableBsm:
      messages.append(("BSM", 1))

    if CP.carFingerprint in RADAR_ACC_CAR and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not CP.flags & ToyotaFlags.SMART_DSU.value:
        messages += [
          ("ACC_CONTROL", 33),
        ]
      messages += [
        ("PCS_HUD", 1),
      ]

    if CP.carFingerprint not in (TSS2_CAR - RADAR_ACC_CAR) and not CP.enableDsu and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      messages += [
        ("PRE_COLLISION", 33),
      ]

    params = Params()
    if params.get_bool('dp_toyota_zss'):
      messages.append(("SECONDARY_STEER_ANGLE", 0))

    if params.get_bool('dp_toyota_enhanced_bsm'):
      messages.append(("DEBUG", 65))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []

    if CP.carFingerprint != CAR.PRIUS_V:
      messages += [
        ("LKAS_HUD", 1),
      ]

    if CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      messages += [
        ("PRE_COLLISION", 33),
        ("ACC_CONTROL", 33),
        ("PCS_HUD", 1),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
