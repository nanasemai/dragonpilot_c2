from cereal import car
from openpilot.common.conversions import Conversions as CV
from panda import Panda
from panda.python import uds
from openpilot.selfdrive.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                        MIN_ACC_SPEED, EPS_SCALE, UNSUPPORTED_DSU_CAR, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.disable_ecu import disable_ecu
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

# dp
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
SteerControlType = car.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    """
    获取车辆参数配置
    参数:
      ret: 返回的车辆参数对象
      candidate: 车型
      fingerprint: 车辆指纹
      car_fw: 车辆固件信息
      experimental_long: 是否启用实验性纵向控制
      docs: 文档信息
    """
    # 设置基本车辆信息
    ret.carName = "toyota"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # BRAKE_MODULE is on a different address for these cars
    # 对于使用新MC PT的车型，制动模块地址不同
    if DBC[candidate]["pt"] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_ALT_BRAKE

    # 角度控制车型的特殊配置
    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_LTA
      # LTA控制可能有更大的延迟
      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      # 配置转向扭矩参数
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    ret.stoppingControl = False  # Toyota starts braking more when it thinks you want to stop

    stop_and_go = candidate in TSS2_CAR

    ret.dashcamOnly = False if ret.dashcamOnly and Params().get_bool("dp_car_dashcam_mode_removal") else ret.dashcamOnly

    # 检测是否存在 smartDSU（智能DSU）
    # smartDSU 可以拦截来自 DSU 或雷达的 ACC_CMD，允许 openpilot 发送控制指令
    # Detect smartDSU, which intercepts ACC_CMD from the DSU (or radar) allowing openpilot to send it
    # 0x2AA is sent by a similar device which intercepts the radar instead of DSU on NO_DSU_CARs
    if 0x2FF in fingerprint[0] or (0x2AA in fingerprint[0] and candidate in NO_DSU_CAR):
      ret.flags |= ToyotaFlags.SMART_DSU.value
    # 对于无 DSU 车型，设置雷达 CAN 过滤标志
    if 0x2AA in fingerprint[0] and candidate in NO_DSU_CAR:
      ret.flags |= ToyotaFlags.RADAR_CAN_FILTER.value
    # TSS2 车型使用摄像头进行纵向控制
    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]
    ret.enableDsu = len(found_ecus) > 0 and Ecu.dsu not in found_ecus and candidate not in (NO_DSU_CAR | UNSUPPORTED_DSU_CAR) \
                    and not (ret.flags & ToyotaFlags.SMART_DSU)
	  # 各车型特定参数配置
    if Ecu.hybrid in found_ecus:
        ret.flags |= ToyotaFlags.HYBRID.value
    if candidate == CAR.PRIUS:
      stop_and_go = True
      ret.wheelbase = 2.70 # 轴距
      ret.steerRatio = 15.74   # 转向比
      ret.tireStiffnessFactor = 0.6371   # 轮胎刚度因子
      ret.mass = 3045. * CV.LB_TO_KG # 车重（磅转千克）
      # 对于角度传感器不良的 Prius，添加转向角死区
      # Only give steer angle deadzone to for bad angle sensor prius
      for fw in car_fw:
        if fw.ecu == "eps" and not fw.fwVersion == b'8965B47060\x00\x00\x00\x00\x00\x00':
          ret.steerActuatorDelay = 0.25
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)

    elif candidate == CAR.PRIUS_V:
      stop_and_go = True
      ret.wheelbase = 2.78
      ret.steerRatio = 17.4
      ret.tireStiffnessFactor = 0.5533
      ret.mass = 3340. * CV.LB_TO_KG

    elif candidate in (CAR.RAV4, CAR.RAV4H):
      stop_and_go = True if (candidate in CAR.RAV4H) else False
      ret.wheelbase = 2.65
      ret.steerRatio = 16.88   # 14.5 is spec end-to-end
      ret.tireStiffnessFactor = 0.5533
      ret.mass = 3650. * CV.LB_TO_KG  # mean between normal and hybrid

    elif candidate == CAR.COROLLA:
      ret.wheelbase = 2.70
      ret.steerRatio = 18.27
      ret.tireStiffnessFactor = 0.444  # not optimized yet
      ret.mass = 2860. * CV.LB_TO_KG  # mean between normal and hybrid

    elif candidate in (CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.79
      ret.steerRatio = 16.  # 14.8 is spec end-to-end
      ret.wheelSpeedFactor = 1.035
      ret.tireStiffnessFactor = 0.5533
      ret.mass = 4481. * CV.LB_TO_KG  # mean between min and max

    elif candidate in (CAR.CHR, CAR.CHR_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.63906
      ret.steerRatio = 13.6
      ret.tireStiffnessFactor = 0.7933
      ret.mass = 3300. * CV.LB_TO_KG

    elif candidate in (CAR.CAMRY, CAR.CAMRY_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.82448
      ret.steerRatio = 13.7
      ret.tireStiffnessFactor = 0.7933
      ret.mass = 3400. * CV.LB_TO_KG  # mean between normal and hybrid

    elif candidate in (CAR.HIGHLANDER, CAR.HIGHLANDER_TSS2):
      # TODO: TSS-P models can do stop and go, but unclear if it requires sDSU or unplugging DSU
      stop_and_go = True
      ret.wheelbase = 2.8194  # average of 109.8 and 112.2 in
      ret.steerRatio = 16.0
      ret.tireStiffnessFactor = 0.8
      ret.mass = 4516. * CV.LB_TO_KG  # mean between normal and hybrid

    elif candidate in (CAR.AVALON, CAR.AVALON_2019, CAR.AVALON_TSS2):
      # starting from 2019, all Avalon variants have stop and go
      # https://engage.toyota.com/static/images/toyota_safety_sense/TSS_Applicability_Chart.pdf
      stop_and_go = candidate != CAR.AVALON
      ret.wheelbase = 2.82
      ret.steerRatio = 14.8  # Found at https://pressroom.toyota.com/releases/2016+avalon+product+specs.download
      ret.tireStiffnessFactor = 0.7983
      ret.mass = 3505. * CV.LB_TO_KG  # mean between normal and hybrid

    elif candidate in (CAR.RAV4_TSS2, CAR.RAV4_TSS2_2022, CAR.RAV4_TSS2_2023):
      ret.wheelbase = 2.68986
      ret.steerRatio = 14.3
      ret.tireStiffnessFactor = 0.7933
      ret.mass = 3585. * CV.LB_TO_KG  # Average between ICE and Hybrid
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kpBP = [0.0]
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.1]
      ret.lateralTuning.pid.kf = 0.00007818594

      # 2019+ RAV4 TSS2 uses two different steering racks and specific tuning seems to be necessary.
      # See https://github.com/commaai/openpilot/pull/21429#issuecomment-873652891
      for fw in car_fw:
        if fw.ecu == "eps" and (fw.fwVersion.startswith(b'\x02') or fw.fwVersion in [b'8965B42181\x00\x00\x00\x00\x00\x00']):
          ret.lateralTuning.pid.kpV = [0.15]
          ret.lateralTuning.pid.kiV = [0.05]
          ret.lateralTuning.pid.kf = 0.00004
          break

    elif candidate == CAR.COROLLA_TSS2:
      ret.wheelbase = 2.67  # Average between 2.70 for sedan and 2.64 for hatchback
      ret.steerRatio = 13.9
      ret.tireStiffnessFactor = 0.444  # not optimized yet
      ret.mass = 3060. * CV.LB_TO_KG

    elif candidate in (CAR.LEXUS_ES, CAR.LEXUS_ES_TSS2):
      ret.wheelbase = 2.8702
      ret.steerRatio = 16.0  # not optimized
      ret.tireStiffnessFactor = 0.444  # not optimized yet
      ret.mass = 3677. * CV.LB_TO_KG  # mean between min and max

    elif candidate == CAR.SIENNA:
      stop_and_go = True
      ret.wheelbase = 3.03
      ret.steerRatio = 15.5
      ret.tireStiffnessFactor = 0.444
      ret.mass = 4590. * CV.LB_TO_KG

    elif candidate in (CAR.LEXUS_IS, CAR.LEXUS_IS_TSS2, CAR.LEXUS_RC):
      ret.wheelbase = 2.79908
      ret.steerRatio = 13.3
      ret.tireStiffnessFactor = 0.444
      ret.mass = 3736.8 * CV.LB_TO_KG

    elif candidate == CAR.LEXUS_GS_F:
      ret.wheelbase = 2.84988
      ret.steerRatio = 13.3
      ret.tireStiffnessFactor = 0.444
      ret.mass = 4034. * CV.LB_TO_KG

    elif candidate == CAR.LEXUS_CTH:
      stop_and_go = True
      ret.wheelbase = 2.60
      ret.steerRatio = 18.6
      ret.tireStiffnessFactor = 0.517
      ret.mass = 3108 * CV.LB_TO_KG  # mean between min and max

    elif candidate in (CAR.LEXUS_NX, CAR.LEXUS_NX_TSS2):
      stop_and_go = True
      ret.wheelbase = 2.66
      ret.steerRatio = 14.7
      ret.tireStiffnessFactor = 0.444  # not optimized yet
      ret.mass = 4070 * CV.LB_TO_KG

    elif candidate == CAR.PRIUS_TSS2:
      ret.wheelbase = 2.70002  # from toyota online sepc.
      ret.steerRatio = 13.4   # True steerRatio from older prius
      ret.tireStiffnessFactor = 0.6371   # hand-tune
      ret.mass = 3115. * CV.LB_TO_KG

    elif candidate == CAR.MIRAI:
      stop_and_go = True
      ret.wheelbase = 2.91
      ret.steerRatio = 14.8
      ret.tireStiffnessFactor = 0.8
      ret.mass = 4300. * CV.LB_TO_KG

    elif candidate == CAR.ALPHARD_TSS2:
      ret.wheelbase = 3.00
      ret.steerRatio = 14.2
      ret.tireStiffnessFactor = 0.444
      ret.mass = 4305. * CV.LB_TO_KG

    ret.centerToFront = ret.wheelbase * 0.44

    # 检测是否启用盲点监测系统(BSM)
    # 某些TSS-P平台的BSM信号会因地区或行驶方向而反转
    # 需要检测信号反转并为C-HR等车型启用
    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR

    # 对于没有DSU且不是TSS2.0的车型，没有雷达DBC文件
    # TODO: 需要为无DSU车型创建ADAS DBC文件
    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make an adas dbc file for dsu-less models
    ret.radarUnavailable = DBC[candidate]['radar'] is None or candidate in (NO_DSU_CAR - TSS2_CAR)

    # 如果检测到smartDSU，openpilot可以发送ACC_CONTROL命令
    # smartDSU会阻止该命令传递给DSU或雷达
    # 由于目前还不能解析TSS2/TSS-P基于雷达的ACC车型的雷达数据
    # 所以将纵向控制功能放在实验性开关后面
    # if the smartDSU is detected, openpilot can send ACC_CONTROL and the smartDSU will block it from the DSU or radar.
    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    use_sdsu = bool(ret.flags & ToyotaFlags.SMART_DSU)
    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      # 实验性纵向控制可用条件：安装了smartDSU或车型在RADAR_ACC_CAR列表中
      ret.experimentalLongitudinalAvailable = use_sdsu or candidate in RADAR_ACC_CAR

      if not use_sdsu:
        # 仅TSS2雷达ACC车型支持禁用雷达
        # Disabling radar is only supported on TSS2 radar-ACC cars
        if experimental_long and candidate in RADAR_ACC_CAR:
          ret.flags |= ToyotaFlags.DISABLE_RADAR.value
      else:
        use_sdsu = use_sdsu and experimental_long

    # openpilot默认启用纵向控制的情况:
    #  - 安装了smartDSU的非TSS2雷达ACC车型
    #  - DSU已断开的车型
    #  - 可以阻止相机发送ACC_CONTROL的TSS2车型
    # openpilot需要实验性开关才能启用纵向控制的情况:
    #  - 安装了smartDSU的TSS2雷达ACC车型
    #  - 未安装smartDSU的TSS2雷达ACC车型(会禁用雷达)
    #  - 安装了CAN过滤器的无DSU的TSS-P车型(暂无雷达解析器)
    # openpilot longitudinal enabled by default:
    #  - non-(TSS2 radar ACC cars) w/ smartDSU installed
    #  - cars w/ DSU disconnected
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    # openpilot longitudinal behind experimental long toggle:
    #  - TSS2 radar ACC cars w/ smartDSU installed
    #  - TSS2 radar ACC cars w/o smartDSU installed (disables radar)
    #  - TSS-P DSU-less cars w/ CAN filter installed (no radar parser yet)
    ret.openpilotLongitudinalControl = use_sdsu or ret.enableDsu or candidate in (TSS2_CAR - RADAR_ACC_CAR) or bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value)
    # 自动恢复停走功能：在支持的车型上启用纵向控制时可用
    ret.autoResumeSng = ret.openpilotLongitudinalControl and candidate in NO_STOP_TIMER_CAR
    # 油门干预器：当存在0x201信号且启用纵向控制时可用
    ret.enableGasInterceptor = 0x201 in fingerprint[0] and ret.openpilotLongitudinalControl
    # 如果不使用openpilot纵向控制，则使用车辆原厂纵向控制
    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_STOCK_LONGITUDINAL
    # 如果启用油门干预器，设置相应的安全参数
    if ret.enableGasInterceptor:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_TOYOTA_GAS_INTERCEPTOR

    # 设置ACC启用的最低速度
    # 如果车辆支持停走功能(stop_and_go)或启用了油门干预器(enableGasInterceptor)
    # 则将启用速度设为负值(-1)，这样速度限制就不会生效
    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else MIN_ACC_SPEED

    # on stock Toyota this is -2.5
    ret.stopAccel = -2.5
    # 纵向控制参数配置
    tune = ret.longitudinalTuning
    tune.deadzoneBP = [0., 16., 20., 30.] # 死区断点速度
    tune.deadzoneV = [.04, .05, .08, .15]  # 对应死区值
    ret.stoppingDecelRate = 0.17  # This is okay for TSS-P  # TSS-P 的停车减速率
    # TSS2 车型特殊配置
    if candidate in TSS2_CAR:
      ret.vEgoStopping = 0.25  # 停车时的速度阈值
      ret.vEgoStarting = 0.25  # 起步时的速度阈值
      ret.stoppingDecelRate = max(0.009, ret.stoppingDecelRate)  # 确保减速率不会太小
      ret.stopAccel = min(-2.0, ret.stopAccel)  # 确保停车加速度不会太大
	    # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05
    # PID 控制器参数
    tune.kpBP = [0., 5.]  # 比例项断点
    tune.kpV = [0.8, 1.]  # 比例系数
    tune.kiBP = [0., 5.]  # 积分项断点
    tune.kiV = [0.3, 1.]  # 积分系数

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    # 如果在没有CAN过滤器/smartDSU的雷达ACC车辆上启用了实验性纵向控制，则禁用雷达
    # disable radar if alpha longitudinal toggled on radar-ACC car without CAN filter/smartDSU
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(logcan, sendcan, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  # returns a car.CarState
  def _update(self, c):
    # 更新车辆状态
    ret = self.CS.update(self.cp, self.cp_cam)
    # 处理车距按钮事件（仅适用于特定车型）
    if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or (self.CP.flags & ToyotaFlags.SMART_DSU and not self.CP.flags & ToyotaFlags.RADAR_CAN_FILTER):
      ret.buttonEvents = create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise})

    # 创建通用事件
    events = self.create_common_events(ret)

    # LTA控制不可用检查
    # 在更准确的角度传感器信号初始化之前，车道跟踪辅助控制不可用
    # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
    # the more accurate angle sensor signal is initialized
    if self.CP.steerControlType == SteerControlType.angle and not self.CS.accurate_steer_angle_seen:
      events.add(EventName.vehicleSensorsInvalid)

    # 纵向控制相关事件检查
    if self.CP.openpilotLongitudinalControl:
      # 停车后需要恢复
      if ret.cruiseState.standstill and not ret.brakePressed and not self.CP.enableGasInterceptor:
        events.add(EventName.resumeRequired)
      # 低速锁定
      if self.CS.low_speed_lockout:
        events.add(EventName.lowSpeedLockout)
      # 速度过低检查
      if ret.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.belowEngageSpeed)
        if c.actuators.accel > 0.3:
          # some margin on the actuator to not false trigger cancellation while stopping
          events.add(EventName.speedTooLow)
        if ret.vEgo < 0.001:
          # while in standstill, send a user alert
          events.add(EventName.manualRestart)

    ret.events = events.to_msg()

    return ret

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
