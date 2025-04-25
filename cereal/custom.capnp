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

struct CustomReserved4 @0x80ae746ee2596b11 {
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
