from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                          create_gas_interceptor_command, make_can_msg
from openpilot.selfdrive.car.toyota import toyotacan
from openpilot.selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# 类型定义
SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
GearShifter = car.CarState.GearShifter

# 转向控制相关常量
MAX_STEER_RATE = 100  # deg/s, EPS故障阈值，如果转向率超过此值且持续施加扭矩会触发故障
MAX_STEER_RATE_FRAMES = 18  # 转向率超限允许的最大帧数，超过此帧数会切断扭矩输出
MAX_USER_TORQUE = 500  # 用户转向扭矩阈值，EPS允许用户施加此阈值以上扭矩的最大帧数为50帧
MAX_LTA_ANGLE = 94.9461  # deg, LTA系统最大转向角度，超过此角度EPS会忽略命令并导致PCS故障
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # LTA模式下允许的最大驾驶员扭矩，略高于方向盘按压力以允许变道时的阻力

# PCM补偿计算阈值
# 在高速时加速命令的变化更明显，让补偿力在应用前降至零
COMPENSATORY_CALCULATION_THRESHOLD_V = [-0.3, -0.25, 0.]  # m/s^2, 加速度补偿阈值
COMPENSATORY_CALCULATION_THRESHOLD_BP = [0., 11., 23.]  # m/s, 速度断点，用于插值计算

# 自动门锁相关CAN命令和阈值
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'  # 车门解锁CAN命令
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'    # 车门上锁CAN命令
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS  # 自动上锁触发速度阈值(10km/h)

# 盲点监测(BSM)相关常量
LEFT_BLINDSPOT = b'\x41'   # 左侧BSM传感器地址标识
RIGHT_BLINDSPOT = b'\x42'  # 右侧BSM传感器地址标识

# 添加加速度限制常量
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

# BSM辅助函数
def set_blindspot_debug_mode(lr, enable):
    """设置BSM调试模式
    Args:
        lr: 左/右BSM传感器标识
        enable: 是否启用调试模式
    Returns:
        CAN消息
    """
    if enable:
        m = lr + b'\x02\x10\x60\x00\x00\x00\x00'  # 启用BSM调试模式的命令
    else:
        m = lr + b'\x02\x10\x01\x00\x00\x00\x00'  # 禁用BSM调试模式的命令
    return make_can_msg(0x750, m, 0)

def poll_blindspot_status(lr):
    """查询BSM状态
    Args:
        lr: 左/右BSM传感器标识
    Returns:
        CAN消息
    """
    m = lr + b'\x02\x21\x69\x00\x00\x00\x00'  # BSM状态查询命令
    return make_can_msg(0x750, m, 0)

class CarController:
  def __init__(self, dbc_name, CP, VM):
    # 基本参数初始化
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_name)
    # 状态变量初始化
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0
    self.distance_button = 0
    self.prohibit_neg_calculation = True
    self.gas = 0
    self.steer_saturation_counter = 0  # 饱和计数器
    self.accel = 0
    # 添加加速度控制相关变量
    self.prev_accel = 0
    self.permit_braking = True

    # 自动门锁相关初始化
    self.dp_toyota_auto_lock_gear_prev = GearShifter.park
    self.dp_toyota_auto_lock_once = False
    p = Params()
    self.dp_toyota_auto_lock = p.get_bool("dp_toyota_auto_lock")
    self.dp_toyota_auto_unlock = p.get_bool("dp_toyota_auto_unlock")
    self.dp_toyota_sng = p.get_bool("dp_toyota_sng")

    # 转向安全余量参数初始化
    steer_rate_safety_margin_str = p.get("dp_toyota_steer_rate_safety_margin", encoding="utf8")
    try:
      self.steer_rate_safety_margin = int(steer_rate_safety_margin_str) if steer_rate_safety_margin_str is not None else 10
      self.steer_rate_safety_margin = max(10, min(200, self.steer_rate_safety_margin))  # 限制在10-200范围内
    except (ValueError, TypeError):
      self.steer_rate_safety_margin = 10  # 默认安全余量

    # 盲点监测相关初始化
    self.dp_toyota_enhanced_bsm = p.get_bool("dp_toyota_enhanced_bsm")
    self._blindspot_debug_enabled_left = False
    self._blindspot_debug_enabled_right = False
    # 为左右两侧设置独立计时器，避免信号冲突
    self._blindspot_frame_left = 0
    self._blindspot_frame_right = 0

    # TSS2相关配置
    if self.CP.carFingerprint in TSS2_CAR: # tss2 can do higher hz then tss1 and can be on at all speed/standstill
      self._blindspot_rate = 2
      self._blindspot_always_on = True
    else:
      self._blindspot_rate = 20
      self._blindspot_always_on = False
    self._last_bsm_poll_left = 0
    self._last_bsm_poll_right = 0
    self._bsm_poll_interval = 1.0 / self._blindspot_rate  # 转换为秒
    self.last_update_nanos = 0
    self._jitter_issue_logged = False


  def update(self, CC, CS, now_nanos):
    # Jitter Monitoring
    if self.last_update_nanos > 0:
        dt_ms = (now_nanos - self.last_update_nanos) / 1e6
        if dt_ms > 30: # Warning if > 30ms (3x the expected 10ms)
            if not self._jitter_issue_logged:
                cloudlog.warning(f"Toyota LKAS: Jitter Detected - dt: {dt_ms:.2f}ms")
                self._jitter_issue_logged = True
        else:
            self._jitter_issue_logged = False
    self.last_update_nanos = now_nanos
    pcm_cancel_cmd = 0
    stopping = CC.actuators.longControlState == LongCtrlState.stopping

    # *** control msgs ***
    can_sends = []

    # 处理车门锁止
    self._handle_door_locks(CS, can_sends)

    # 处理盲点监测
    self._handle_blindspot_monitoring(CS, can_sends)

    # *** steer torque ***
    # 非横向激活状态下禁用转向
    if not CC.latActive:
      apply_steer = 0
      apply_steer_req = False
    else:
      # 转向率安全检查 (使用参数化的安全余量)
      if abs(CS.out.steeringRateDeg) > MAX_STEER_RATE + self.steer_rate_safety_margin:
        apply_steer = 0
        apply_steer_req = False
        #cloudlog.warning(f"Toyota LKAS: Steer Rate Limit Exceeded: {CS.out.steeringRateDeg:.1f} deg/s (Margin: {self.steer_rate_safety_margin})")
      else:
        # 计算转向扭矩并应用限制
        new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
        apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # 调用故障预防函数
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    # *** steer angle 转向角度 ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      # 如果使用LTA控制，禁用LKA并设置转向角命令
      apply_steer = 0
      apply_steer_req = False
      # 初始化apply_angle以避免未定义错误
      apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        # EPS使用扭矩传感器角度进行控制，偏移进行补偿
        apply_angle = CC.actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed 基于速度的角速度限制
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        # LTA转向响应优化
        #lta_active = CC.latActive and self.CP.steerControlType == SteerControlType.angle
        #if lta_active and self.CP.carFingerprint in TSS2_CAR:
          # 根据车速动态调整转向响应
          # 根据车速动态调整转向响应（steer_rate_limit 当前未使用，保留供后续优化）
          # steer_rate_limit = interp(CS.out.vEgo, [0, 10, 20], [100, 50, 25])
          # 注意：移除了直接乘以steer_rate_scale的逻辑，因为apply_std_steer_angle_limits已经处理了角度变化率限制
          # 避免了直接修改目标角度导致的蛇形摆动问题

        if not CC.latActive:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

      # 确保在所有分支下都能安全使用apply_angle
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
      lta_active = CC.latActive and self.CP.steerControlType == SteerControlType.angle

      # 优化的扭矩渐变控制
      if lta_active:
        full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX * 0.9 and  # 降低阈值到90%
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE * 0.9)

        # 更平滑的扭矩渐变
        torque_wind_down = 100 if full_torque_condition else \
                          interp(abs(CS.out.steeringTorqueEps),
                                [self.params.STEER_MAX * 0.6,  # 提前开始渐变
                                 self.params.STEER_MAX * 0.9], # 降低最大阈值
                                [100, 0])
      else:
        apply_angle = CS.out.steeringAngleDeg
        torque_wind_down = 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType,
                                                         self.last_angle, lta_active,
                                                         self.frame // 2, torque_wind_down))
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
      pedal_command = PEDAL_SCALE * (CC.actuators.accel + pedal_offset)
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
    if (self.CP.carFingerprint in NO_STOP_TIMER_CAR and CC.actuators.accel < 1e-3 or stopping) or CS.out.vEgo < 1e-3:
      should_compensate = False
    # limit minimum to only positive until first positive is reached after engagement, don't calculate when long isn't active
    if CC.longActive and should_compensate and not self.prohibit_neg_calculation:
      accel_offset = CS.pcm_neutral_force / self.CP.mass
    else:
      accel_offset = 0.
    # only calculate pcm_accel_cmd when long is active to prevent disengagement from accelerator depression
    if CC.longActive:
      # 1. 先计算补偿
      pcm_accel_cmd = CC.actuators.accel + accel_offset
      # 2. 应用范围限制
      pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      # 3. 应用速率限制（如果需要）
      if self.CP.openpilotLongitudinalControl:
        pcm_accel_cmd = clip(
          pcm_accel_cmd - self.prev_accel,
          ACCEL_WINDDOWN_LIMIT,
          ACCEL_WINDUP_LIMIT
        ) + self.prev_accel
      # 统一更新prev_accel
      self.prev_accel = pcm_accel_cmd
    else:
      pcm_accel_cmd = 0.
      self.prev_accel = 0.

    # 制动许可逻辑改进
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        if pcm_accel_cmd < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif pcm_accel_cmd > 0.3:
          self.permit_braking = False

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
    hud_control = CC.hudControl
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
      accel_raw = -2.5 if stopping else CC.actuators.accel

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
                                                     hud_control.rightLaneDepart, CC.latActive, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append([0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00", 0])

    new_actuators = CC.actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends

  def _handle_door_locks(self, CS, can_sends):
    """处理车门自动锁止/解锁逻辑"""
    try:
      if not CS.out.doorOpen and CS.out.gearShifter is not None:
        gear = CS.out.gearShifter
        if gear == GearShifter.park and self.dp_toyota_auto_lock_gear_prev != gear and CS.out.vEgo is not None:
          if self.dp_toyota_auto_unlock and abs(CS.out.vEgo) < 0.1:
            can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
            self.dp_toyota_auto_lock_once = False
        elif gear == GearShifter.drive and not self.dp_toyota_auto_lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
          if self.dp_toyota_auto_lock and not CS.out.brakePressed:
            can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
            self.dp_toyota_auto_lock_once = True
        self.dp_toyota_auto_lock_gear_prev = gear
    except Exception:
      cloudlog.exception("Error in door lock control")
      self.dp_toyota_auto_lock_once = False

  def _handle_blindspot_monitoring(self, CS, can_sends):
    """处理盲点监测系统(BSM)逻辑"""
    if not self.dp_toyota_enhanced_bsm:
      return

    # 处理左侧BSM
    if not self._blindspot_debug_enabled_left:
      if (self._blindspot_always_on or (CS.out.leftBlinker and CS.out.vEgo > 6)):
        can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, True))
        self._blindspot_debug_enabled_left = True
    else:
      if not self._blindspot_always_on and not CS.out.leftBlinker and self.frame - self._blindspot_frame_left > 50:
        can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, False))
        self._blindspot_debug_enabled_left = False
      if self.frame % self._blindspot_rate == 0:
        can_sends.append(poll_blindspot_status(LEFT_BLINDSPOT))
        if CS.out.leftBlinker:
          self._blindspot_frame_left = self.frame

    # 处理右侧BSM
    if not self._blindspot_debug_enabled_right:
      if (self._blindspot_always_on or (CS.out.rightBlinker and CS.out.vEgo > 6)):
        can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, True))
        self._blindspot_debug_enabled_right = True
    else:
      if not self._blindspot_always_on and not CS.out.rightBlinker and self.frame - self._blindspot_frame_right > 50:
        can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, False))
        self._blindspot_debug_enabled_right = False
      if self.frame % self._blindspot_rate == self._blindspot_rate // 2:
        can_sends.append(poll_blindspot_status(RIGHT_BLINDSPOT))
        if CS.out.rightBlinker:
          self._blindspot_frame_right = self.frame
