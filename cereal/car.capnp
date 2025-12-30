using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0x8e2af1e708af8b8d;

# ******* events causing controls state machine transition *******

struct CarEvent @0x9b1657f34caf3ad3 {
  name @0 :EventName; # 事件名称

  # event types
  enable @1 :Bool; # 启用自动驾驶
  noEntry @2 :Bool; # 禁止进入自动驾驶状态
  warning @3 :Bool;   # 仅在启用或软禁用时显示的警告
  userDisable @4 :Bool; # 用户手动禁用
  softDisable @5 :Bool; # 软禁用（平滑退出）
  immediateDisable @6 :Bool; # 立即禁用（紧急退出）
  preEnable @7 :Bool; # 预启用状态
  permanent @8 :Bool; # 无论openpilot状态如何都显示的警告
  overrideLateral @10 :Bool; # 用户接管横向控制
  overrideLongitudinal @9 :Bool; # 用户接管纵向控制

  enum EventName @0xbaa8c5d505f727de {
    canError @0; # CAN总线错误
    steerUnavailable @1; # 转向不可用
    wrongGear @4; # 错误档位
    doorOpen @5; # 车门打开
    seatbeltNotLatched @6; # 安全带未系
    espDisabled @7; # ESP（电子稳定程序）已禁用
    wrongCarMode @8; # 错误的车辆模式
    steerTempUnavailable @9; # 转向临时不可用
    reverseGear @10; # 倒档
    buttonCancel @11; # 取消按钮
    buttonEnable @12; # 启用按钮
    pedalPressed @13;  # 踏板被踩下（退出激活状态）
    preEnableStandstill @73;  # 带刹车的预启用状态
    gasPressedOverride @108;  # 用户踩油门但无油门退出功能时
    steerOverride @114; # 用户接管转向
    cruiseDisabled @14; # 巡航禁用
    speedTooLow @17; # 速度过低
    outOfSpace @18; # 存储空间不足
    overheat @19; # 过热
    calibrationIncomplete @20; # 校准不完整
    calibrationInvalid @21; # 校准无效
    controlsMismatch @22; # 控制不匹配
    pcmEnable @23; # PCM启用
    pcmDisable @24; # PCM禁用
    radarFault @26; # 雷达故障
    brakeHold @28; # 刹车保持
    parkBrake @29; # 驻车制动
    manualRestart @30; # 手动重启
    lowSpeedLockout @31; # 低速锁定
    plannerError @32; # 规划器错误
    joystickDebug @34; # 摇杆调试
    steerTempUnavailableSilent @35; # 转向临时不可用（静默）
    resumeRequired @36; # 需要恢复
    preDriverDistracted @37; # 驾驶员分心预警
    promptDriverDistracted @38; # 提示驾驶员分心
    driverDistracted @39; # 驾驶员分心
    preDriverUnresponsive @43; # 驾驶员无响应预警
    promptDriverUnresponsive @44; # 提示驾驶员无响应
    driverUnresponsive @45; # 驾驶员无响应
    belowSteerSpeed @46; # 低于转向速度
    lowBattery @48; # 电池电量低
    vehicleModelInvalid @50; # 车辆模型无效
    accFaulted @51; # ACC（自适应巡航）故障
    sensorDataInvalid @52; # 传感器数据无效
    commIssue @53; # 通信问题
    commIssueAvgFreq @109; # 通信问题平均频率
    tooDistracted @54; # 过于分心
    posenetInvalid @55; # 姿态估计无效
    soundsUnavailable @56; # 声音不可用
    preLaneChangeLeft @57; # 准备左变道
    preLaneChangeRight @58; # 准备右变道
    laneChange @59; # 变道
    lowMemory @63; # 内存不足
    stockAeb @64; # 原厂AEB（自动紧急制动）
    ldw @65; # LDW（车道偏离警告）
    carUnrecognized @66; # 车辆未识别
    invalidLkasSetting @69; # LKAS（车道保持辅助系统）设置无效
    speedTooHigh @70; # 速度过高
    laneChangeBlocked @71; # 变道被阻止
    relayMalfunction @72; # 继电器故障
    stockFcw @74; # 原厂FCW（前碰撞预警）
    startup @75; # 启动
    startupNoCar @76; # 启动但无车辆
    startupNoControl @77; # 启动但无控制
    startupMaster @78; # 主启动
    startupNoFw @104; # 启动但无固件
    fcw @79; # FCW（前碰撞预警）
    steerSaturated @80; # 转向饱和
    belowEngageSpeed @84; # 低于激活速度
    noGps @85; # 无GPS信号
    wrongCruiseMode @87; # 错误的巡航模式
    modeldLagging @89; # 模型计算延迟
    deviceFalling @90; # 设备坠落
    fanMalfunction @91; # 风扇故障
    cameraMalfunction @92; # 摄像头故障
    cameraFrameRate @110; # 摄像头帧率
    gpsMalfunction @94; # GPS故障
    processNotRunning @95; # 进程未运行
    dashcamMode @96; # 行车记录仪模式
    controlsInitializing @98; # 控制初始化
    usbError @99; # USB错误
    roadCameraError @100; # 道路摄像头错误
    driverCameraError @101; # 驾驶员摄像头错误
    wideRoadCameraError @102; # 广角道路摄像头错误
    localizerMalfunction @103; # 定位器故障
    highCpuUsage @105; # CPU使用率高
    cruiseMismatch @106; # 巡航不匹配
    lkasDisabled @107; # LKAS（车道保持辅助系统）已禁用
    canBusMissing @111; # CAN总线缺失
    controlsdLagging @112; # 控制延迟
    resumeBlocked @113; # 恢复被阻止
    steerTimeLimit @115; # 转向时间限制
    vehicleSensorsInvalid @116; # 车辆传感器无效
    calibrationRecalibrating @117; # 校准重新校准中
    roadEdgeDetected @118;  # 道路边缘检测提醒
    steerTorqueOver @119;  # 力矩过大
    leadStartAlert @120;  # 前车起步提醒

    radarCanErrorDEPRECATED @15; # 雷达CAN错误（已弃用）
    communityFeatureDisallowedDEPRECATED @62; # 社区功能不允许（已弃用）
    radarCommIssueDEPRECATED @67; # 雷达通信问题（已弃用）
    driverMonitorLowAccDEPRECATED @68; # 驾驶员监控低精度（已弃用）
    gasUnavailableDEPRECATED @3; # 油门不可用（已弃用）
    dataNeededDEPRECATED @16; # 需要数据（已弃用）
    modelCommIssueDEPRECATED @27; # 模型通信问题（已弃用）
    ipasOverrideDEPRECATED @33; # IPAS接管（已弃用）
    geofenceDEPRECATED @40; # 地理围栏（已弃用）
    driverMonitorOnDEPRECATED @41; # 驾驶员监控开启（已弃用）
    driverMonitorOffDEPRECATED @42; # 驾驶员监控关闭（已弃用）
    calibrationProgressDEPRECATED @47; # 校准进度（已弃用）
    invalidGiraffeHondaDEPRECATED @49; # 无效的Giraffe Honda（已弃用）
    invalidGiraffeToyotaDEPRECATED @60; # 无效的Giraffe Toyota（已弃用）
    internetConnectivityNeededDEPRECATED @61; # 需要互联网连接（已弃用）
    whitePandaUnsupportedDEPRECATED @81; # 白色Panda不支持（已弃用）
    commIssueWarningDEPRECATED @83; # 通信问题警告（已弃用）
    focusRecoverActiveDEPRECATED @86; # 注意力恢复活动（已弃用）
    neosUpdateRequiredDEPRECATED @88; # 需要Neos更新（已弃用）
    modelLagWarningDEPRECATED @93; # 模型延迟警告（已弃用）
    startupOneplusDEPRECATED @82; # OnePlus启动（已弃用）
    startupFuzzyFingerprintDEPRECATED @97; # 启动模糊指纹（已弃用）
    noTargetDEPRECATED @25; # 无目标（已弃用）
    brakeUnavailableDEPRECATED @2; # 刹车不可用（已弃用）

  }
}

# ******* main car state @ 100hz *******
# all speeds in m/s

struct CarState {
  events @13 :List(CarEvent);

  # CAN健康状态
  canValid @26 :Bool;       # 无效计数器/校验和
  canTimeout @40 :Bool;     # CAN总线断开
  canErrorCounter @48 :UInt32;  # CAN错误计数器

  # 车速
  vEgo @1 :Float32;            # 最佳速度估计值
  aEgo @16 :Float32;           # 最佳加速度估计值
  vEgoRaw @17 :Float32;        # 来自轮速传感器的未滤波速度
  vEgoCluster @44 :Float32;    # 车辆仪表盘显示的速度估计值，用于UI

  vCruise @53 :Float32;        # 实际设置的巡航速度
  vCruiseCluster @54 :Float32; # UI中显示的巡航速度

  yawRate @22 :Float32;     # 最佳偏航率估计值
  standstill @18 :Bool;     # 车辆静止状态
  wheelSpeeds @2 :WheelSpeeds;  # 轮速信息

  # 油门踏板，0.0-1.0
  gas @3 :Float32;        # 用户油门踏板位置
  gasPressed @4 :Bool;    # 用户是否踩下油门

  engineRpm @46 :Float32;  # 发动机转速

  # 制动踏板，0.0-1.0
  brake @5 :Float32;      # 用户制动踏板位置
  brakePressed @6 :Bool;  # 用户是否踩下制动
  regenBraking @45 :Bool; # 是否启用再生制动
  parkingBrake @39 :Bool; # 驻车制动状态
  brakeHoldActive @38 :Bool; # 制动保持功能激活状态

  # 方向盘
  steeringAngleDeg @7 :Float32; # 方向盘角度（度）
  steeringAngleOffsetDeg @37 :Float32; # 多个传感器之间的角度偏移
  steeringRateDeg @15 :Float32;    # 方向盘角速度（度/秒）
  steeringTorque @8 :Float32;      # 方向盘扭矩（原生CAN单位）
  steeringTorqueEps @27 :Float32;  # EPS方向盘扭矩（原生CAN单位）
  steeringPressed @9 :Bool;        # 用户是否正在干预方向盘
  steeringDisengage @58 :Bool;     # 比steeringPressed更大的力，适用于特定品牌的退出条件
  steerFaultTemporary @35 :Bool;   # 转向系统临时故障
  steerFaultPermanent @36 :Bool;   # 转向系统永久性故障

  invalidLkasSetting @55 :Bool;    # 原厂LKAS配置错误（开或关）
  stockAeb @30 :Bool;        # 原厂自动紧急制动功能状态
  stockLkas @59 :Bool;       # 原厂车道保持辅助系统状态
  stockFcw @31 :Bool;        # 原厂前方碰撞预警功能状态
  espDisabled @32 :Bool;     # 电子稳定程序是否禁用
  accFaulted @42 :Bool;      # 自适应巡航系统故障
  carFaultedNonCritical @47 :Bool;  # 某些ECU故障，但车辆仍可控制
  espActive @51 :Bool;       # 电子稳定程序是否激活
  vehicleSensorsInvalid @52 :Bool;  # 车辆传感器读数无效（如方向盘角度等）
  lowSpeedAlert @56 :Bool;  # 由于动态最小转向速度导致失去转向控制
  blockPcmEnable @60 :Bool;  # 是否允许PCM启用此帧

  # 巡航状态
  cruiseState @10 :CruiseState;  # 巡航系统状态

  # 档位
  gearShifter @14 :GearShifter;  # 换挡杆位置

  # 按钮事件
  buttonEvents @11 :List(ButtonEvent);  # 按钮事件列表
  buttonEnable @57 :Bool;  # 用户请求启用，通常只有一帧。当pcmCruise=False时设置
  leftBlinker @20 :Bool;   # 左转向灯状态
  rightBlinker @21 :Bool;  # 右转向灯状态
  genericToggle @23 :Bool; # 通用切换开关状态

  # 锁止信息
  doorOpen @24 :Bool;      # 车门是否打开
  seatbeltUnlatched @25 :Bool;  # 安全带是否未系

  # 离合器（仅手动变速箱）
  clutchPressed @28 :Bool;  # 离合器是否踩下

  # 盲点传感器
  leftBlindspot @33 :Bool; # 左侧车道变换是否有障碍物
  rightBlindspot @34 :Bool; # 右侧车道变换是否有障碍物

  fuelGauge @41 :Float32; # 电池或燃油箱水平（0.0到1.0）
  charging @43 :Bool;     # 是否正在充电

  # 处理元数据
  cumLagMs @50 :Float32;  # 累计延迟时间(毫秒)

  vCluRatio @61 :Float32;  # 车速传感器比率
  logCarrot @62 :Text;  # 记录的路径点
  softHoldActive @63 :Int16;    # 0: 未激活, 1: 准备激活, 2: 已激活
  activateCruise @64 :Int16;  # 激活巡航控制
  latEnabled @65 :Bool;  # 横向控制启用状态
  pcmCruiseGap @66 :Int16;      # 0: 无法读取, 1,2,3,4: 车距设置
  speedLimit @67 :Float32;  # 限速值
  speedLimitDistance @68 :Float32;  # 距离限速标志的距离
  gearStep @69 :Int16;          # 档位步进值
  tpms @70 : Tpms;  # 胎压监测系统信息
  useLaneLineSpeed @71 : Float32;  # 使用车道线速度
  leftLatDist @72 : Float32;  # 到左侧车道线的横向距离
  rightLatDist @73 : Float32; # 到右侧车道线的横向距离
  leftLongDist @74 : Float32; # 沿行驶方向到左侧车道线的纵向距离
  rightLongDist @75 : Float32; # 沿行驶方向到右侧车道线的纵向距离
  carrotCruise @76 : Int16;  # 智能巡航控制
  leftLaneLine @77 : Int16; # -1: 无车道线, 0: 虚线, 1: 实线, +10: 白色, +20: 黄色, 例如) 21: 黄色实线
  rightLaneLine @78 : Int16; # -1: 无车道线, 0: 虚线, 1: 实线, +10: 白色, +20: 黄色, 例如) 21: 黄色实线
  datetime @79 :UInt64; # 从纪元开始的毫秒时间戳

  struct Tpms {
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct WheelSpeeds {
    # optional wheel speeds
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct CruiseState {
    enabled @0 :Bool;  # 巡航控制启用状态
    speed @1 :Float32;  # 巡航设定速度
    speedCluster @6 :Float32;  # 仪表盘显示的设定速度
    available @2 :Bool;  # 巡航功能可用状态
    speedOffset @3 :Float32;  # 速度偏移量
    standstill @4 :Bool;  # 车辆静止状态
    nonAdaptive @5 :Bool;  # 非自适应巡航模式
  }

  enum GearShifter {  # 档位状态枚举
    unknown @0;  # 未知
    park @1;  # 驻车档
    drive @2;  # 前进档
    neutral @3;  # 空档
    reverse @4;  # 倒档
    sport @5;  # 运动模式
    low @6;  # 低速档
    brake @7;  # 刹车档
    eco @8;  # 经济模式
    manumatic @9;  # 手动模式
  }

  # 按钮状态变化时发送
  struct ButtonEvent {
    pressed @0 :Bool;  # 按钮是否被按下
    type @1 :Type;  # 按钮类型

    enum Type {  # 按钮类型枚举
      unknown @0;  # 未知按钮
      leftBlinker @1;  # 左转向灯
      rightBlinker @2;  # 右转向灯
      accelCruise @3;  # 加速巡航
      decelCruise @4;  # 减速巡航
      cancel @5;  # 取消按钮
      altButton1 @6;  # 备选按钮1
      altButton2 @7;  # 备选按钮2
      altButton3 @8;  # 备选按钮3
      setCruise @9;  # 设置巡航速度
      resumeCruise @10;  # 恢复巡航
      gapAdjustCruise @11;  # 调整巡航车距
    }
  }

  # deprecated
  errorsDEPRECATED @0 :List(CarEvent.EventName);
  brakeLightsDEPRECATED @19 :Bool;
  steeringRateLimitedDEPRECATED @29 :Bool;
  canMonoTimesDEPRECATED @12: List(UInt64);
  canRcvTimeoutDEPRECATED @49 :Bool;
}
# ******* radar state @ 20hz *******
struct RadarData @0x888ad6581cf0aacb {  # 雷达数据结构体
  errors @0 :List(Error);  # 雷达错误信息列表
  points @1 :List(RadarPoint);  # 雷达探测点列表
  enum Error {  # 雷达错误类型枚举
    canError @0;  # CAN总线错误
    fault @1;  # 雷达故障
    wrongConfig @2;  # 雷达配置错误
  }
  # similar to LiveTracks
  # is one timestamp valid for all? I think so
  struct RadarPoint {  # 雷达探测点结构体
    trackId @0 :UInt64;  # no trackId reuse  # 目标跟踪ID（不可重复使用）

    # these 3 are the minimum required
    # 以下3个是最基本的必需字段
    dRel @1 :Float32; # m from the front bumper of the car  # 相对距离（从汽车前保险杠开始的米数）
    yRel @2 :Float32; # m  # 横向相对距离（米）
    vRel @3 :Float32; # m/s  # 相对速度（米/秒）

    # these are optional and valid if they are not NaN
    # 以下是可选字段，非NaN值表示有效
    aRel @4 :Float32; # m/s^2  # 相对加速度（米/秒²）
    yvRel @5 :Float32; # m/s  # 横向相对速度（米/秒）

    # some radars flag measurements VS estimates
    # 某些雷达会标记是测量值还是估计值
    measured @6 :Bool;  # 是否为测量值
  }

  # deprecated
  # 已废弃字段
  canMonoTimesDEPRECATED @2 :List(UInt64);
}

# ******* car controls @ 100hz *******

struct CarControl {  # 车辆控制结构体
  # must be true for any actuator commands to work
  # 必须为true才能使任何执行器命令生效
  enabled @0 :Bool;  # 控制启用状态
  latActive @11: Bool;  # 横向控制激活状态
  longActive @12: Bool;  # 纵向控制激活状态
  # Actuator commands as computed by controlsd
  # 由controlsd计算的执行器命令
  actuators @6 :Actuators;  # 执行器控制命令
  leftBlinker @15: Bool;  # 左转向灯
  rightBlinker @16: Bool;  # 右转向灯
  # Any car specific rate limits or quirks applied by
  # the CarController are reflected in actuatorsOutput
  # and matches what is sent to the car
  # 任何车型特定的速率限制或特殊处理都反映在actuatorsOutput中
  # 与发送给汽车的命令一致
  actuatorsOutput @10 :Actuators;  # 实际发送给汽车的执行器命令
  orientationNED @13 :List(Float32);  # 东北天坐标系下的方向
  angularVelocity @14 :List(Float32);  # 角速度
  cruiseControl @4 :CruiseControl;  # 巡航控制
  hudControl @5 :HUDControl;  # 抬头显示控制

  struct Actuators {  # 执行器控制结构体
    # range from 0.0 - 1.0
    # 范围从0.0到1.0
    gas @0: Float32;  # 油门指令
    brake @1: Float32;  # 制动指令
    # range from -1.0 - 1.0
    # 范围从-1.0到1.0
    steer @2: Float32;  # 转向指令
    # value sent over can to the car
    # 通过CAN发送给汽车的值
    steerOutputCan @8: Float32;  # 实际发送到CAN的转向值
    steeringAngleDeg @3: Float32;  # 转向角度（度）
    curvature @7: Float32;  # 曲率
    speed @6: Float32; # m/s  # 速度（米/秒）
    accel @4: Float32; # m/s^2  # 加速度（米/秒²）
    longControlState @5: LongControlState;  # 纵向控制状态

    enum LongControlState @0xe40f3a917d908282{  # 纵向控制状态枚举
      off @0;  # 关闭
      pid @1;  # PID控制
      stopping @2;  # 停车中
      starting @3;  # 起步中
    }
  }

  struct CruiseControl {  # 巡航控制结构体
    cancel @0: Bool;  # 取消巡航
    resume @1: Bool;  # 恢复巡航
    override @4: Bool;  # 驾驶员干预巡航
    speedOverrideDEPRECATED @2: Float32;  # 速度覆盖（已废弃）
    accelOverrideDEPRECATED @3: Float32;  # 加速度覆盖（已废弃）
  }

  struct HUDControl {  # 抬头显示控制结构体
    speedVisible @0: Bool;  # 速度显示可见性
    setSpeed @1: Float32;  # 设置的速度
    lanesVisible @2: Bool;  # 车道线显示可见性
    leadVisible @3: Bool;  # 前车显示可见性
    visualAlert @4: VisualAlert;  # 视觉警报
    audibleAlert @5: AudibleAlert;  # 声音警报
    rightLaneVisible @6: Bool;  # 右侧车道线显示可见性
    leftLaneVisible @7: Bool;  # 左侧车道线显示可见性
    rightLaneDepart @8: Bool;  # 右侧车道偏离
    leftLaneDepart @9: Bool;  # 左侧车道偏离
    leadDistanceBars @10: Int8;  # 1-3: 1 is closest, 3 is farthest. some ports may utilize 2-4 bars instead  # 前车距离条（1-3：1最近，3最远。某些端口可能使用2-4条）

    enum VisualAlert {  # 视觉警报枚举
      # these are the choices from the Honda
      # map as good as you can for your car
      none @0;  # 无警报
      fcw @1;  # 前方碰撞预警
      steerRequired @2;  # 需要驾驶员接管转向
      brakePressed @3;  # 制动已踩下
      wrongGear @4;  # 档位错误
      seatbeltUnbuckled @5;  # 安全带未系
      speedTooHigh @6;  # 速度过高
      ldw @7;  # 车道偏离预警
    }

    enum AudibleAlert {  # 声音警报枚举
      none @0;  # 无警报
      engage @1;  # 系统激活
      disengage @2;  # 系统解除
      refuse @3;  # 系统拒绝操作

      warningSoft @4;  # 轻微警告
      warningImmediate @5;  # 紧急警告

      prompt @6;  # 提示音
      promptRepeat @7;  # 重复提示音
      promptDistracted @8;  # 分心提示音
    }
  }

  gasDEPRECATED @1 :Float32;  # 油门（已废弃）
  brakeDEPRECATED @2 :Float32;  # 制动（已废弃）
  steeringTorqueDEPRECATED @3 :Float32;  # 转向扭矩（已废弃）
  activeDEPRECATED @7 :Bool;  # 激活状态（已废弃）
  rollDEPRECATED @8 :Float32;  # 侧倾（已废弃）
  pitchDEPRECATED @9 :Float32;  # 俯仰（已废弃）
}

# op new version

struct CarOutput {
  # Any car specific rate limits or quirks applied by
  # the CarController are reflected in actuatorsOutput
  # and matches what is sent to the car
  actuatorsOutput @0 :CarControl.Actuators;
}

# ****** car param ******

struct CarParams {  # 车辆参数结构体
  carName @0 :Text;  # 车辆名称
  carFingerprint @1 :Text;  # 车辆指纹（用于识别车型）
  fuzzyFingerprint @55 :Bool;  # 模糊匹配指纹

  notCar @66 :Bool;  # flag for non-car robotics platforms  # 非车辆机器人平台标志

  enableGasInterceptor @2 :Bool;  # 启用油门拦截器
  pcmCruise @3 :Bool;        # is openpilot's state tied to the PCM's cruise state?  # openpilot状态是否与PCM巡航状态绑定
  enableDsu @5 :Bool;        # driving support unit  # 启用驾驶支持单元
  enableBsm @56 :Bool;       # blind spot monitoring  # 启用盲点监测
  flags @64 :UInt32;         # flags for car specific quirks  # 车型特定特性标志
  experimentalLongitudinalAvailable @71 :Bool;  # 实验性纵向控制可用性

  minEnableSpeed @7 :Float32;  # 最小启用速度
  minSteerSpeed @8 :Float32;  # 最小转向速度
  safetyConfigs @62 :List(SafetyConfig);  # 安全配置列表
  alternativeExperience @65 :Int16;      # panda flag for features like no disengage on gas  # Panda特殊功能标志（如加油不退出）

  # Car docs fields
  # 车辆文档字段
  maxLateralAccel @68 :Float32;  # 最大横向加速度
  autoResumeSng @69 :Bool;               # describes whether car can resume from a stop automatically  # 描述车辆是否能自动从停止状态恢复

  # things about the car in the manual
  # 车辆手册中的信息
  mass @17 :Float32;            # [kg] curb weight: all fluids no cargo  # [千克]整备质量：满油无水无货物
  wheelbase @18 :Float32;       # [m] distance from rear axle to front axle  # [米]轴距：后轴到前轴的距离
  centerToFront @19 :Float32;   # [m] distance from center of mass to front axle  # [米]质心到前轴的距离
  steerRatio @20 :Float32;      # [] ratio of steering wheel angle to front wheel angle  # []转向传动比：方向盘角度与前轮角度的比值
  steerRatioRear @21 :Float32;  # [] ratio of steering wheel angle to rear wheel angle (usually 0)  # []后转向传动比：方向盘角度与后轮角度的比值（通常为0）

  # things we can derive
  # 可推导的参数
  rotationalInertia @22 :Float32;    # [kg*m2] body rotational inertia  # [千克·米²]车身转动惯量
  tireStiffnessFactor @72 :Float32;  # scaling factor used in calculating tireStiffness[Front,Rear]  # 用于计算轮胎刚度的缩放因子
  tireStiffnessFront @23 :Float32;   # [N/rad] front tire coeff of stiff  # [牛/弧度]前轮胎刚度系数
  tireStiffnessRear @24 :Float32;    # [N/rad] rear tire coeff of stiff  # [牛/弧度]后轮胎刚度系数

  longitudinalTuning @25 :LongitudinalPIDTuning;  # 纵向PID控制参数
  lateralParams @48 :LateralParams;  # 横向控制参数
  lateralTuning :union {  # 横向控制调优方式（联合类型）
    pid @26 :LateralPIDTuning;  # PID控制调优
    indi @27 :LateralINDITuning;  # 增量非干扰控制调优
    lqr @40 :LateralLQRTuning;  # 线性二次调节器控制调优
    torque @67 :LateralTorqueTuning;  # 扭矩控制调优
  }

  steerLimitAlert @28 :Bool;  # 转向限制警报
  steerLimitTimer @47 :Float32;  # time before steerLimitAlert is issued  # 触发转向限制警报前的时间

  vEgoStopping @29 :Float32; # Speed at which the car goes into stopping state  # 车辆进入停止状态的速度
  vEgoStarting @59 :Float32; # Speed at which the car goes into starting state  # 车辆进入起步状态的速度
  stoppingControl @31 :Bool; # Does the car allow full control even at lows speeds when stopping  # 车辆在低速停车时是否允许完全控制
  steerControlType @34 :SteerControlType;  # 转向控制类型
  radarUnavailable @35 :Bool; # True when radar objects aren't visible on CAN or aren't parsed out  # 当雷达目标在CAN总线上不可见或未解析时为True
  stopAccel @60 :Float32; # Required acceleration to keep vehicle stationary  # 保持车辆静止所需的加速度
  stoppingDecelRate @52 :Float32; # m/s^2/s while trying to stop  # 停车时的减速度变化率（米/秒³）
  startAccel @32 :Float32; # Required acceleration to get car moving  # 使车辆开始移动所需的加速度
  startingState @70 :Bool; # Does this car make use of special starting state  # 车辆是否使用特殊的起步状态

  steerActuatorDelay @36 :Float32; # Steering wheel actuator delay in seconds  # 方向盘执行器延迟（秒）
  longitudinalActuatorDelayLowerBound @61 :Float32; # Gas/Brake actuator delay in seconds, lower bound  # 油门/制动执行器延迟（秒），下限
  longitudinalActuatorDelayUpperBound @58 :Float32; # Gas/Brake actuator delay in seconds, upper bound  # 油门/制动执行器延迟（秒），上限
  openpilotLongitudinalControl @37 :Bool; # is openpilot doing the longitudinal control?  # openpilot是否负责纵向控制
  carVin @38 :Text; # VIN number queried during fingerprinting  # 指纹识别期间查询的车辆识别号码
  dashcamOnly @41: Bool;  # 仅行车记录仪模式
  transmissionType @43 :TransmissionType;  # 变速箱类型
  carFw @44 :List(CarFw);  # 车辆固件信息列表

  radarTimeStep @45: Float32 = 0.05;  # time delta between radar updates, 20Hz is very standard  # 雷达更新之间的时间间隔，通常为20Hz
  fingerprintSource @49: FingerprintSource;  # 指纹来源
  networkLocation @50 :NetworkLocation;  # Where Panda/C2 is integrated into the car's CAN network  # Panda/C2在车辆CAN网络中的集成位置

  wheelSpeedFactor @63 :Float32; # Multiplier on wheels speeds to computer actual speeds  # 车轮速度计算实际速度的乘数

  useLongitudinalTuner @73 :Bool; # custom  # 使用纵向调谐器（自定义）

  struct SafetyConfig {
    safetyModel @0 :SafetyModel;
    safetyParam @3 :UInt16;
    safetyParamDEPRECATED @1 :Int16;
    safetyParam2DEPRECATED @2 :UInt32;
  }

  struct LateralParams {
    torqueBP @0 :List(Int32);
    torqueV @1 :List(Int32);
  }

  struct LateralPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @4 :Float32;
  }

  struct LateralTorqueTuning {
    useSteeringAngle @0 :Bool;
    kp @1 :Float32;
    ki @2 :Float32;
    friction @3 :Float32;
    kf @4 :Float32;
    steeringAngleDeadzoneDeg @5 :Float32;
    latAccelFactor @6 :Float32;
    latAccelOffset @7 :Float32;
  }

  struct LongitudinalPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @6 :Float32;
    deadzoneBP @4 :List(Float32);
    deadzoneV @5 :List(Float32);
  }

  struct LateralINDITuning {
    outerLoopGainBP @4 :List(Float32);
    outerLoopGainV @5 :List(Float32);
    innerLoopGainBP @6 :List(Float32);
    innerLoopGainV @7 :List(Float32);
    timeConstantBP @8 :List(Float32);
    timeConstantV @9 :List(Float32);
    actuatorEffectivenessBP @10 :List(Float32);
    actuatorEffectivenessV @11 :List(Float32);

    outerLoopGainDEPRECATED @0 :Float32;
    innerLoopGainDEPRECATED @1 :Float32;
    timeConstantDEPRECATED @2 :Float32;
    actuatorEffectivenessDEPRECATED @3 :Float32;
  }

  struct LateralLQRTuning {
    scale @0 :Float32;
    ki @1 :Float32;
    dcGain @2 :Float32;

    # State space system
    a @3 :List(Float32);
    b @4 :List(Float32);
    c @5 :List(Float32);

    k @6 :List(Float32);  # LQR gain
    l @7 :List(Float32);  # Kalman gain
  }

  enum SafetyModel {
    silent @0;
    hondaNidec @1;
    toyota @2;
    elm327 @3;
    gm @4;
    hondaBoschGiraffe @5;
    ford @6;
    cadillac @7;
    hyundai @8;
    chrysler @9;
    tesla @10;
    subaru @11;
    gmPassive @12;
    mazda @13;
    nissan @14;
    volkswagen @15;
    toyotaIpas @16;
    allOutput @17;
    gmAscm @18;
    noOutput @19;  # like silent but without silent CAN TXs
    hondaBosch @20;
    volkswagenPq @21;
    subaruPreglobal @22;  # pre-Global platform
    hyundaiLegacy @23;
    hyundaiCommunity @24;
    volkswagenMlb @25;
    hongqi @26;
    body @27;
    hyundaiCanfd @28;
    volkswagenMqbEvo @29;
    byd @30;
  }

  enum SteerControlType {
    torque @0;
    angle @1;

    curvatureDEPRECATED @2;
  }

  enum TransmissionType {
    unknown @0;
    automatic @1;  # Traditional auto, including DSG
    manual @2;  # True "stick shift" only
    direct @3;  # Electric vehicle or other direct drive
    cvt @4;
  }

  struct CarFw {
    ecu @0 :Ecu;
    fwVersion @1 :Data;
    address @2 :UInt32;
    subAddress @3 :UInt8;
    responseAddress @4 :UInt32;
    request @5 :List(Data);
    brand @6 :Text;
    bus @7 :UInt8;
    logging @8 :Bool;
    obdMultiplexing @9 :Bool;
  }

  enum Ecu {
    eps @0;
    abs @1;
    fwdRadar @2;
    fwdCamera @3;
    engine @4;
    unknown @5;
    transmission @8; # Transmission Control Module
    hybrid @18; # hybrid control unit, e.g. Chrysler's HCP, Honda's IMA Control Unit, Toyota's hybrid control computer
    srs @9; # airbag
    gateway @10; # can gateway
    hud @11; # heads up display
    combinationMeter @12; # instrument cluster
    electricBrakeBooster @15;
    shiftByWire @16;
    adas @19;
    cornerRadar @21;
    hvac @20;
    parkingAdas @7;  # parking assist system ECU, e.g. Toyota's IPAS, Hyundai's RSPA, etc.
    epb @22;  # electronic parking brake
    telematics @23;
    body @24;  # body control module

    # Toyota only
    dsu @6;

    # Honda only
    vsa @13; # Vehicle Stability Assist
    programmedFuelInjection @14;

    debug @17;
  }

  enum FingerprintSource {
    can @0;
    fw @1;
    fixed @2;
  }

  enum NetworkLocation {
    fwdCamera @0;  # Standard/default integration at LKAS camera
    gateway @1;    # Integration at vehicle's CAN gateway
  }

  enableCameraDEPRECATED @4 :Bool;
  enableApgsDEPRECATED @6 :Bool;
  steerRateCostDEPRECATED @33 :Float32;
  isPandaBlackDEPRECATED @39 :Bool;
  hasStockCameraDEPRECATED @57 :Bool;
  safetyParamDEPRECATED @10 :Int16;
  safetyModelDEPRECATED @9 :SafetyModel;
  safetyModelPassiveDEPRECATED @42 :SafetyModel = silent;
  minSpeedCanDEPRECATED @51 :Float32;
  communityFeatureDEPRECATED @46: Bool;
  startingAccelRateDEPRECATED @53 :Float32;
  steerMaxBPDEPRECATED @11 :List(Float32);
  steerMaxVDEPRECATED @12 :List(Float32);
  gasMaxBPDEPRECATED @13 :List(Float32);
  gasMaxVDEPRECATED @14 :List(Float32);
  brakeMaxBPDEPRECATED @15 :List(Float32);
  brakeMaxVDEPRECATED @16 :List(Float32);
  directAccelControlDEPRECATED @30 :Bool;
  maxSteeringAngleDegDEPRECATED @54 :Float32;
}
