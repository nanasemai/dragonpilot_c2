using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# you can rename the struct, but don't change the identifier
struct LiveMapData @0x81c2f05a394cf4af {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;
  speedLimitAheadValid @2 :Bool;
  speedLimitAhead @3 :Float32;
  speedLimitAheadDistance @4 :Float32;
  turnSpeedLimitValid @5 :Bool;
  turnSpeedLimit @6 :Float32;
  turnSpeedLimitEndDistance @7 :Float32;
  turnSpeedLimitSign @8 :Int16;
  turnSpeedLimitsAhead @9 :List(Float32);
  turnSpeedLimitsAheadDistances @10 :List(Float32);
  turnSpeedLimitsAheadSigns @11 :List(Int16);
  lastGpsTimestamp @12 :Int64;  # Milliseconds since January 1, 1970.
  currentRoadName @13 :Text;
  lastGpsLatitude @14 :Float64;
  lastGpsLongitude @15 :Float64;
  lastGpsSpeed @16 :Float32;
  lastGpsBearingDeg @17 :Float32;
  lastGpsAccuracy @18 :Float32;
  lastGpsBearingAccuracyDeg @19 :Float32;
}

struct LongitudinalPlanExt @0xaedffd8f31e7b55d {
  visionTurnControllerState @0 :VisionTurnControllerState;
  visionTurnSpeed @1 :Float32;
  speedLimitControlState @2 :SpeedLimitControlState;
  speedLimit @3 :Float32;
  speedLimitOffset @4 :Float32;
  distToSpeedLimit @5 :Float32;
  isMapSpeedLimit @6 :Bool;
  speedLimitPercOffset @7 :Bool;
  speedLimitValueOffset @8 :Float32;
  distToTurn @9 :Float32;
  turnSpeed @10 :Float32;
  turnSpeedControlState @11 :SpeedLimitControlState;
  turnSign @12 :Int16;
  dpE2EIsBlended @13 :Bool;
  longitudinalPlanExtSource @14 :LongitudinalPlanExtSource;
  # 添加 ACM 相关状态
  acmEnabled @15 :Bool;      # ACM 功能是否启用
  acmDownhillOnly @16 :Bool; # 是否仅在下坡时启用 ACM
  acmActive @17 :Bool;      # ACM 是否处于激活状态
  desiredFollowDistance @18 :Float32; # 添加计算得到的期望跟车距离
  leadStartAlert @19 :Bool;  # 前车起步提醒状态

  enum LongitudinalPlanExtSource {
    cruise @0;
    lead0 @1;
    lead1 @2;
    lead2 @3;
    e2e @4;
    turn @5;
    limit @6;
    turnlimit @7;
  }

  enum SpeedLimitControlState {
    inactive @0; # No speed limit set or not enabled by parameter.
    tempInactive @1; # User wants to ignore speed limit until it changes.
    adapting @2; # Reducing speed to match new speed limit.
    active @3; # Cruising at speed limit.
  }

  enum VisionTurnControllerState {
    disabled @0; # No predicted substancial turn on vision range or feature disabled.
    entering @1; # A subsantial turn is predicted ahead, adapting speed to turn confort levels.
    turning @2; # Actively turning. Managing acceleration to provide a roll on turn feeling.
    leaving @3; # Road ahead straightens. Start to allow positive acceleration.
  }
}

struct LateralPlanExt @0xf35cc4560bbf6ec2 {
  dPathWLinesX @0 :List(Float32);
  dPathWLinesY @1 :List(Float32);
}

struct ControlsStateExt @0xda96579883444c35 {
  alkaActive @0 :Bool;
  alkaEnabled @1 :Bool;
  lateralState @2 :Text;
}

struct CarrotMan @0x80ae746ee2596b11 {
	activeCarrot @0 : Int32;     # 激活的导航点索引
	nRoadLimitSpeed @1 : Int32; # 道路限速值
	remote @2 : Text;           # 远程控制相关信息
	xSpdType @3 : Int32;        # 速度类型
	xSpdLimit @4 : Int32;       # 速度限制值
	xSpdDist @5 : Int32;        # 距离速度限制点的距离
	xSpdCountDown @6 : Int32;   # 速度限制倒计时
	xTurnInfo @7 : Int32;       # 转弯信息
	xDistToTurn @8 : Int32;     # 距离转弯点的距离
	xTurnCountDown @9 : Int32;  # 转弯倒计时
	atcType @10 : Text;         # 自动巡航类型
	vTurnSpeed @11 : Int32;     # 转弯速度
	szPosRoadName @12 : Text;   # 当前道路名称
	szTBTMainText @13 : Text;   # 主要的转弯提示文本
	desiredSpeed @14 : Int32;   # 期望速度
	desiredSource @15 : Text;   # 期望速度的来源
	carrotCmdIndex @16 : Int32; # 导航命令索引
	carrotCmd @17 : Text;       # 导航命令
	carrotArg @18 : Text;       # 导航命令参数
	xPosLat @19 : Float32;      # 纬度坐标
	xPosLon @20 : Float32;      # 经度坐标
	xPosAngle @21 : Float32;    # 位置角度
	xPosSpeed @22 : Float32;    # 位置处的速度
	trafficState @23 : Int32;   # 交通状态
	nGoPosDist @24 : Int32;     # 目标位置距离
	nGoPosTime @25 : Int32;     # 到达目标位置的时间
	szSdiDescr @26 : Text;      # 特殊驾驶信息描述
	naviPaths @27 : Text;       # 导航路径信息
	leftSec @28 : Int32;        # 剩余时间（秒）
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}
