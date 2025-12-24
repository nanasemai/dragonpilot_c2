import numpy as np
from cereal import car
from openpilot.common.conversions import Conversions as CV
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
    # 初始化CAN总线定义和车辆参数
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.  # 电动助力转向扭矩比例
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.  # 仪表盘速度滞后间隙
    self.cluster_min_speed = CV.KPH_TO_MS / 2.  # 仪表盘最小速度

    # 转向角度相关初始化
    # 在某些车型上，转向角度信号会在启动时归零
    # 需要在收到两个转向角度测量值后应用偏移量
    self.accurate_steer_angle_seen = False  # 是否已获得准确的转向角度
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)  # 转向角度偏移滤波器

    # 车距控制按钮状态
    self.prev_distance_button = 0  # 上一次车距按钮状态
    self.distance_button = 0  # 当前车距按钮状态
    self.pcm_follow_distance = 0  # PCM跟车距离

    # 系统状态标志
    self.low_speed_lockout = False  # 低速锁定标志
    self.acc_type = 1  # ACC类型
    self.lkas_hud = {}  # LKAS抬头显示信息

    # ZSS(零点转向传感器)相关参数
    params = Params()
    self._dp_alka = params.get_bool('dp_alka')  # ALKA功能开关
    self._dp_toyota_zss = params.get_bool('dp_toyota_zss')  # ZSS功能开关
    self._dp_zss_compute = False  # ZSS计算标志
    self._dp_zss_cruise_active_last = False  # 上一次巡航状态
    self._dp_zss_angle_offset = 0.  # ZSS角度偏移
    self._dp_zss_threshold_count = 0  # ZSS阈值计数器

    # BSM(盲点监测)相关参数
    self.dp_toyota_enhanced_bsm = params.get_bool('dp_toyota_enhanced_bsm')  # 增强型BSM开关
    # 左侧盲点监测
    self._left_blindspot = False  # 左侧盲点状态
    self._left_blindspot_d1 = 0  # 左侧盲点距离1
    self._left_blindspot_d2 = 0  # 左侧盲点距离2
    self._left_blindspot_counter = 0  # 左侧盲点计数器

    # 右侧盲点监测
    self._right_blindspot = False  # 右侧盲点状态
    self._right_blindspot_d1 = 0  # 右侧盲点距离1
    self._right_blindspot_d2 = 0  # 右侧盲点距离2
    self._right_blindspot_counter = 0  # 右侧盲点计数器

    # BSM超时参数
    self._BSM_COUNTER_MAX = 100  # BSM最大计数
    self._BSM_TIMEOUT = 0.5  # BSM超时时间(秒)
    self._last_bsm_update = 0.  # 最后BSM更新时间

    # 帧计数器
    self.frame = 0  # 初始化帧计数器

  def update(self, cp, cp_cam):
    """更新车辆状态
    Args:
        cp: 主CAN总线解析器
        cp_cam: 相机CAN总线解析器
    Returns:
        ret: 更新后的车辆状态
    """
    ret = car.CarState.new_message()

    # 车身状态检测
    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])  # 车门开启状态
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0  # 安全带解开状态
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1  # 驻车制动状态

    # 制动和油门状态
    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0  # 制动踏板状态
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1  # 制动保持状态

    # 油门干预器状态检测
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2  # 油门干预器数值
      ret.gasPressed = ret.gas > 805  # 油门踩下状态
    else:
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0  # 原厂油门状态

    # 车轮速度处理
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],  # 左前轮速
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],  # 右前轮速
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],  # 左后轮速
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],  # 右后轮速
    )

    # 车速计算和过滤
    wheel_speeds = [ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]
    ret.vEgoRaw = float(np.mean([s for s in wheel_speeds if abs(s) < 150]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)  # 经过卡尔曼滤波的车速和加速度
    ret.vEgoCluster = ret.vEgo * 1.015  # 仪表盘显示车速
    ret.standstill = abs(ret.vEgoRaw) < 1e-3  # 车辆静止状态

    # 添加车辆传感器验证
    if self.CP.steerControlType == SteerControlType.angle:
      ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    # 转向角度处理
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]  # 转向角度
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]  # 转向角速度
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]  # 扭矩传感器角度

    # 转向角度初始化和偏移处理
    # 只要传感器不再初始化，就认为角度已准确（移除角度非零检查，避免方向盘精准停在0度时的误判）
    if not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      # 大转向角和高角速度时不更新偏移
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    # ZSS转向角处理
    if self._dp_zss_threshold_count > ZSS_THRESHOLD_COUNT:
      self._dp_toyota_zss = False
    if self._dp_toyota_zss:
      try:
        # 使用get方法防止KeyError，如果消息或信号不存在则返回默认值0
        zorro_steer = cp.vl.get("SECONDARY_STEER_ANGLE", {}).get("ZORRO_STEER", 0)
        # 合理性检查：注意500度阈值可能需要根据目标车型的转向极限进行调整
        # 某些丰田车型在掉头时转向角可能超过500度
        if abs(zorro_steer) > 500:  # 添加合理性检查
          self._dp_zss_threshold_count = ZSS_THRESHOLD_COUNT + 1
        else:
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
      except Exception:
        self._dp_toyota_zss = False
        cloudlog.exception("ZSS error")

    # 变速箱和转向信号
    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])  # 档位信息
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))  # 档位状态
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1  # 左转向灯
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2  # 右转向灯

    # 发动机转速(除Mirai外的车型)
    if self.CP.carFingerprint != CAR.MIRAI:
      ret.engineRpm = cp.vl["ENGINE_RPM"]["RPM"]

    # 获取转向相关数据
    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]  # 驾驶员施加的转向扭矩
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale  # EPS电机输出的转向扭矩(经过缩放)
    # 我们可以使用dbc中的override位，但它在扭矩值太高时才触发
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD  # 判断是否有方向盘转向操作

    # 检查EPS LKA/LTA故障状态
    lka_state = cp.vl["EPS_STATUS"]["LKA_STATE"]
    ret.steerFaultTemporary = lka_state in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = lka_state in PERM_STEER_FAULTS

    if lka_state != getattr(self, '_prev_lka_state', -1):
        if ret.steerFaultPermanent:
            cloudlog.error(f"Toyota EPS LKA Fault: State changed to {lka_state} (Permanent)")
        elif ret.steerFaultTemporary:
            cloudlog.warning(f"Toyota EPS LKA Fault: State changed to {lka_state} (Temporary)")
        self._prev_lka_state = lka_state

    # 如果是角度控制类型，还需要检查LTA状态
    if self.CP.steerControlType == SteerControlType.angle:
      lta_state = cp.vl["EPS_STATUS"]["LTA_STATE"]
      lta_fault_temp = lta_state in TEMP_STEER_FAULTS
      lta_fault_perm = lta_state in PERM_STEER_FAULTS
      ret.steerFaultTemporary = ret.steerFaultTemporary or lta_fault_temp
      ret.steerFaultPermanent = ret.steerFaultPermanent or lta_fault_perm

      if lta_state != getattr(self, '_prev_lta_state', -1):
          if lta_fault_perm:
              cloudlog.error(f"Toyota EPS LTA Fault: State changed to {lta_state} (Permanent)")
          elif lta_fault_temp:
              cloudlog.warning(f"Toyota EPS LTA Fault: State changed to {lta_state} (Temporary)")
          self._prev_lta_state = lta_state

    # 处理不同车型的巡航控制状态
    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:  # 不支持DSU的车型
      # TODO: 需要在DSU_CRUISE中找到描述ACC故障的位，可能在CLUTCH中也存在
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0  # 巡航系统是否可用
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS  # 设定的巡航速度
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]  # 仪表盘显示的设定速度
    else:  # 支持DSU的车型
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0  # ACC系统是否故障
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0  # 巡航系统是否可用
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS  # 设定的巡航速度
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]  # 仪表盘显示的设定速度

    # 处理仪表盘速度显示
    # UI_SET_SPEED在main开启时总是非零，所以要等到首次启用才显示
    if ret.cruiseState.speed != 0:
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)  # 判断是公制还是英制
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS  # 选择相应的转换系数
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor  # 计算仪表盘显示速度

    # 确定ACC控制源
    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp  # TSS2且无雷达ACC的车型使用相机信号

    # TSS2车型的特殊处理
    if self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]  # 获取ACC类型
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])  # 原厂前碰撞预警系统状态

    # 某些TSS2车型的低速锁定是永久设置的，这些车型通过ACC_TYPE值为2来识别
    # 可以通过在启动时发送自定义的ACC_CONTROL消息（ACC_TYPE设为1）来避免锁定并获得停走功能
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
      (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      # 改进巡航控制故障检测
      if self.CP.openpilotLongitudinalControl:
          ret.accFaulted = ret.accFaulted or cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2  # 低速锁定状态

    # 巡航控制状态处理
    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]  # PCM-ACC状态
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # 某些车型忽略静止状态，因为PCM允许仅通过加速请求就重新启动
      ret.cruiseState.standstill = self.pcm_acc_status == 7  # 车辆静止状态
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])  # 巡航控制是否启用
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)  # 是否为非自适应巡航
    # dp - PCM补偿
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]  # PCM中性力

    # 其他车辆状态
    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])  # 自动远光灯开关状态
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0  # ESP（电子稳定程序）是否禁用

    # 原厂AEB（自动紧急制动）状态
    if not self.CP.enableDsu and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    # BSM（盲点监测）状态
    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)  # 左侧盲点
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)  # 右侧盲点

    # LKAS HUD（抬头显示）更新
    if self.CP.carFingerprint != CAR.PRIUS_V:
      self.lkas_hud = cp_cam.vl["LKAS_HUD"]  # 车道保持辅助系统显示信息 - 直接赋值优化内存使用，避免copy.copy的微小开销

    # dp 车距按钮处理
    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]  # PCM跟车距离

    # TSS2无雷达ACC车型的距离按钮处理
    if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      # 距离按钮连接到ACC模块（相机或雷达）
      self.prev_distance_button = self.distance_button  # 保存上一次距离按钮状态
      if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]  # 更新当前距离按钮状态

    # dp - acc过滤器 - 用于不带雷达的sdsu的距离按钮逻辑
    #self.prev_distance_button, self.distance_button = self.acc_filter_state.get_distance_button_states(self.prev_distance_button, self.distance_button)

    # 启用盲点调试模式（由@arne182开发）
    # 保留注释掉的代码以便将来调试
    if self.dp_toyota_enhanced_bsm and self.frame > 199:  # 增强型BSM功能
      debug_vl = cp.vl.get("DEBUG", {})  # 安全获取DEBUG消息，默认空字典
      distance_1 = debug_vl.get('BLINDSPOTD1')  # 盲点距离1
      distance_2 = debug_vl.get('BLINDSPOTD2')  # 盲点距离2
      side = debug_vl.get('BLINDSPOTSIDE')  # 盲点侧边指示

      # 确保所有盲点监测相关数据都有效
      if distance_1 is not None and distance_2 is not None and side is not None:
        if side == 65:  # 左侧盲点检测
          # 当距离1发生变化时，更新数据并重置计数器
          if distance_1 != self._left_blindspot_d1:
            self._left_blindspot_d1 = distance_1
            self._left_blindspot_counter = 100  # 设置100个周期的检测时间
          # 当距离2发生变化时，更新数据并重置计数器
          if distance_2 != self._left_blindspot_d2:
            self._left_blindspot_d2 = distance_2
            self._left_blindspot_counter = 100
          # 如果任一距离大于10，表示检测到左侧盲点目标
          if self._left_blindspot_d1 > 10 or self._left_blindspot_d2 > 10:
            self._left_blindspot = True

        elif side == 66:  # 右侧盲点检测
          # 当距离1发生变化时，更新数据并重置计数器
          if distance_1 != self._right_blindspot_d1:
            self._right_blindspot_d1 = distance_1
            self._right_blindspot_counter = 100
          # 当距离2发生变化时，更新数据并重置计数器
          if distance_2 != self._right_blindspot_d2:
            self._right_blindspot_d2 = distance_2
            self._right_blindspot_counter = 100
          # 如果任一距离大于10，表示检测到右侧盲点目标
          if self._right_blindspot_d1 > 10 or self._right_blindspot_d2 > 10:
            self._right_blindspot = True

      # 左侧盲点计数器处理（移到条件外，确保无论是否收到新消息都能递减）
      if self._left_blindspot_counter > 0:
        self._left_blindspot_counter -= 1  # 计数器递减
      else:
        # 计数器归零时清除左侧盲点状态
        self._left_blindspot = False
        self._left_blindspot_d1 = 0
        self._left_blindspot_d2 = 0

      # 右侧盲点计数器处理（移到条件外，确保无论是否收到新消息都能递减）
      if self._right_blindspot_counter > 0:
        self._right_blindspot_counter -= 1  # 计数器递减
      else:
        # 计数器归零时清除右侧盲点状态
        self._right_blindspot = False
        self._right_blindspot_d1 = 0
        self._right_blindspot_d2 = 0

      # 更新返回值中的盲点状态
      ret.leftBlindspot = self._left_blindspot
      ret.rightBlindspot = self._right_blindspot

    self.frame += 1
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
