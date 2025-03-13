from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                          create_gas_interceptor_command, make_can_msg
from openpilot.selfdrive.car.toyota import toyotacan
from openpilot.selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# LKA limits (lane keeping assist) 以扭矩控制，只在有需要(接近道路邊線)時修正，也就是反覆來回修正
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
# 如果在转向率高于100度/秒的情况下施加扭矩的时间过长，则EPS故障
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
# EPS允许用户在永久性故障之前将扭矩保持在阈值以上50帧
MAX_USER_TORQUE = 500

# LTA limits （Lane Tracing Assist 车道追踪辅助 ）以轉角控制，隨時進行修正，車道置中
# EPS ignores commands above this angle and causes PCS to fault
# EPS忽略此角度以上的命令，导致PCS故障
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes 稍微高于方向盘的压力会在变道时产生一些阻力

# PCM compensatory force calculation threshold
# PCM补偿力计算阈值
# a variation in accel command is more pronounced at higher speeds, let compensatory forces ramp to zero before
# 加速指令的变化在较高速度下更为明显，让补偿力在之前降至零
# applying when speed is high
# 高速时应用
COMPENSATORY_CALCULATION_THRESHOLD_V = [-0.3, -0.25, 0.]  # m/s^2
COMPENSATORY_CALCULATION_THRESHOLD_BP = [0., 11., 23.]  # m/s

# rick - toyota auto lock / unlock
GearShifter = car.CarState.GearShifter
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS

# Blindspot codes
LEFT_BLINDSPOT = b'\x41'
RIGHT_BLINDSPOT = b'\x42'

def set_blindspot_debug_mode(lr,enable):
  if enable:
    m = lr + b'\x02\x10\x60\x00\x00\x00\x00'
  else:
    m = lr + b'\x02\x10\x01\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)


def poll_blindspot_status(lr):
  m = lr + b'\x02\x21\x69\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0
    self.distance_button = 0
    self.prohibit_neg_calculation = True

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

    self.dp_toyota_auto_lock_gear_prev = GearShifter.park
    self.dp_toyota_auto_lock_once = False
    p = Params()
    self.dp_toyota_auto_lock = p.get_bool("dp_toyota_auto_lock")
    self.dp_toyota_auto_unlock = p.get_bool("dp_toyota_auto_unlock")
    self.dp_toyota_sng = p.get_bool("dp_toyota_sng")

    # dp - bsm
    self.dp_toyota_enhanced_bsm = p.get_bool("dp_toyota_enhanced_bsm")
    self._blindspot_debug_enabled_left = False
    self._blindspot_debug_enabled_right = False
    self._blindspot_frame = 0

    if self.CP.carFingerprint in TSS2_CAR: # tss2 can do higher hz then tss1 and can be on at all speed/standstill
      self._blindspot_rate = 2
      self._blindspot_always_on = True
    else:
      self._blindspot_rate = 20
      self._blindspot_always_on = False
    self._last_bsm_poll_left = 0
    self._last_bsm_poll_right = 0
    self._bsm_poll_interval = 1.0 / self._blindspot_rate  # 转换为秒


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE
    stopping = actuators.longControlState == LongCtrlState.stopping

    # *** control msgs ***
    can_sends = []

    # dp - door auto lock / unlock logic
    # thanks to AlexandreSato & cydia2020
    # https://github.com/AlexandreSato/animalpilot/blob/personal/doors.py
    try:
      if not CS.out.doorOpen and CS.out.gearShifter is not None:
        gear = CS.out.gearShifter
        # 添加速度有效性检查
        if gear == GearShifter.park and self.dp_toyota_auto_lock_gear_prev != gear and CS.out.vEgo is not None:
          if self.dp_toyota_auto_unlock and abs(CS.out.vEgo) < 0.1:  # 静止状态才允许解锁
            can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
            self.dp_toyota_auto_lock_once = False
        elif gear == GearShifter.drive and not self.dp_toyota_auto_lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
          if self.dp_toyota_auto_lock and not CS.out.brakePressed:  # 刹车时不自动上锁
            can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
            self.dp_toyota_auto_lock_once = True
        self.dp_toyota_auto_lock_gear_prev = gear
    except Exception as e:
      cloudlog.exception("Error in door lock control")
      self.dp_toyota_auto_lock_once = False  # 异常时重置状态

    # Enable blindspot debug mode once (@arne182)
    # let's keep all the commented out code for easy debug purpose for future.
    if self.dp_toyota_enhanced_bsm:
      #if self.frame > 200:
      #left bsm
      if not self._blindspot_debug_enabled_left:
        if (self._blindspot_always_on or (CS.out.leftBlinker and CS.out.vEgo > 6)): # eagle eye camera will stop working if right bsm is switched on under 6m/s
          can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, True))
          self._blindspot_debug_enabled_left = True
          # print("bsm debug left, on")
      else:
        if not self._blindspot_always_on and not CS.out.leftBlinker and self.frame - self._blindspot_frame > 50:
          can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, False))
          self._blindspot_debug_enabled_left = False
          # print("bsm debug left, off")
        if self.frame % self._blindspot_rate == 0:
          can_sends.append(poll_blindspot_status(LEFT_BLINDSPOT))
          if CS.out.leftBlinker:
            self._blindspot_frame = self.frame
            # print(self._blindspot_frame)
          # print("bsm poll left")
      #right bsm
      if not self._blindspot_debug_enabled_right:
        if (self._blindspot_always_on or (CS.out.rightBlinker and CS.out.vEgo > 6)): # eagle eye camera will stop working if right bsm is switched on under 6m/s
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, True))
          self._blindspot_debug_enabled_right = True
          # print("bsm debug right, on")
      else:
        if not self._blindspot_always_on and not CS.out.rightBlinker and self.frame - self._blindspot_frame > 50:
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, False))
          self._blindspot_debug_enabled_right = False
          # print("bsm debug right, off")
        if self.frame % self._blindspot_rate == self._blindspot_rate/2:
          can_sends.append(poll_blindspot_status(RIGHT_BLINDSPOT))
          if CS.out.rightBlinker:
            self._blindspot_frame = self.frame
            # print(self._blindspot_frame)
          # print("bsm poll right")

    # *** steer torque 转向扭矩 ***
    if abs(CS.out.steeringRateDeg) > MAX_STEER_RATE + 5:  # 添加安全余量
      apply_steer = 0
      apply_steer_req = False
    else:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention >100度/秒转向故障预防
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # *** steer angle 转向角度 ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      # 如果使用LTA控制，禁用LKA并设置转向角命令
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        # EPS使用扭矩传感器角度进行控制，偏移进行补偿
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed 基于速度的角速度限制
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    # 丰田可以在42Hz下跟踪显示STEERING_LKA，计数器交替添加1和2；
    # 以100Hz的频率发送似乎允许更高的速率限制，因为速率限制似乎是强加的连续消息
    can_sends.append(toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req))

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    # STEERING_LTA似乎不允许通过更快的发送来获得更高的速率，最终可能会更容易
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      # 当EPS扭矩或驾驶员扭矩高于
      # 阈值分别用于限制最大横向加速度和驾驶员扭矩混合。
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      # TORQUE_WIND_DOWN在0时以大约1500单位/秒的最大下降速率降低扭矩
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

    # *** gas and brake ***
    if self.CP.enableGasInterceptor and CC.longActive:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if self.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.

    # prohibit negative compensatory calculations when first activating long after accelerator depression or engagement
    # 在加速器踩下或接合后很久才首次激活时，禁止进行负补偿计算
    if not CC.longActive:
      self.prohibit_neg_calculation = True
    comp_thresh = interp(CS.out.vEgo, COMPENSATORY_CALCULATION_THRESHOLD_BP, COMPENSATORY_CALCULATION_THRESHOLD_V)
    # don't reset until a reasonable compensatory value is reached
    # 在达到合理的补偿值之前，不要重置
    if CS.pcm_neutral_force > comp_thresh * self.CP.mass:
      self.prohibit_neg_calculation = False
    # NO_STOP_TIMER_CAR will creep if compensation is applied when stopping or stopped, don't compensate when stopped or stopping
    # 如果在停止或停止时应用补偿，NO_STOP_TIMER_CAR将爬行，停止或停止后不进行补偿
    should_compensate = True
    if (self.CP.carFingerprint in NO_STOP_TIMER_CAR and actuators.accel < 1e-3 or stopping) or CS.out.vEgo < 1e-3:
      should_compensate = False
    # limit minimum to only positive until first positive is reached after engagement, don't calculate when long isn't active
    if CC.longActive and should_compensate and not self.prohibit_neg_calculation:
      accel_offset = CS.pcm_neutral_force / self.CP.mass
    else:
      accel_offset = 0.
    # only calculate pcm_accel_cmd when long is active to prevent disengagement from accelerator depression
    if CC.longActive:
      pcm_accel_cmd = clip(actuators.accel + accel_offset, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
    else:
      pcm_accel_cmd = 0.

    # TODO: probably can delete this. CS.pcm_acc_status uses a different signal
    # than CS.cruiseState.enabled. confirm they're not meaningfully different
    if not CC.enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False
    if self.dp_toyota_sng:
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    # we can spam can to cancel the system even if we are using lat only control
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged
      # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
      if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
        desired_distance = 4 - hud_control.leadDistanceBars
        if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
          self.distance_button = not self.distance_button
        else:
          self.distance_button = 0

      # dp - for pcm compensation
      # when stopping, send -2.5 raw acceleration immediately to prevent vehicle from creeping, else send actuators.accel
      # 停车时，立即发送-2.5的原始加速度，以防止车辆爬行，否则发送执行器
      accel_raw = -2.5 if stopping else actuators.accel

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        can_sends.append(toyotacan.create_accel_command(self.packer,
														 pcm_accel_cmd,
														 accel_raw,
														 pcm_cancel_cmd,
                             self.standstill_req,
														 lead,
														 CS.acc_type,
														 fcw_alert,
														 self.distance_button))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(toyotacan.create_accel_command(self.packer,
														0,
														0,
														pcm_cancel_cmd,
														False,
														lead,
														CS.acc_type,
														False,
														self.distance_button))

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # 如果gas cmd为零，则发送零。拦截器将发送读取值和gas cmd之间的最大值。
      # This prevents unexpected pedal range rescaling
      # 这可以防止意外的踏板范围重新缩放
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
        (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append([0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00", 0])

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
