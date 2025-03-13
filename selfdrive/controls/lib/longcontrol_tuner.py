from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.hybrid_modeld.constants import ModelConstants

import json
import os
import time

LongCtrlState = car.CarControl.Actuators.LongControlState

def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControlTuner():
  """纵向控制调谐器
    
    功能：
    1. 动态加载和更新PID参数
    2. 实现纵向控制状态管理
    3. 执行PID控制计算
    4. 提供参数在线调整
    """
  def __init__(self, CP):
    """初始化控制器
        
        参数：
        - CP: 车辆参数配置
        """
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    # PID控制相关参数
    self.v_pid = 0.0 # PID控制目标速度
    self.last_output_accel = 0.0 # 上一次输出的加速度
    self.k_f = CP.longitudinalTuning.kf # 前馈增益
    # 死区参数
    self.deadzoneBP = [0.0] # 死区断点
    self.deadzoneV = [0.0] # 死区值
    
    self.pid = None
    # 调谐器配置
    self.tuner_filepath = os.getcwd() + '/../../long_pid_tuner.json'
    self.tuner_last_check_update = time.time()
    self.tuner_modified_time = None
    self.tuner_update_interval = 10 # every 10 seconds # 更新间隔(秒)
    
    print("using LongControlTuner conf:", self.tuner_filepath)
        
    self.reload_tuner() # 加载调谐参数

  def write_tuner(self):
    with open(self.tuner_filepath, 'w') as f:  
      #print("dumping longitudeTuning", type(self.CP.longitudinalTuning.kpBP), type(self.CP.longitudinalTuning.kpV), type(self.CP.longitudinalTuning.kiBP), type(self.CP.longitudinalTuning.kiV), type(self.CP.longitudinalTuning.deadzoneBP), type(self.CP.longitudinalTuning.deadzoneV))
      print("dumping longitudeTuning", self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV, self.CP.longitudinalTuning.kiBP, self.CP.longitudinalTuning.kiV, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      data = {"kp_bp": list(self.CP.longitudinalTuning.kpBP),
              "kp_v": list(self.CP.longitudinalTuning.kpV),
              "ki_bp": list(self.CP.longitudinalTuning.kiBP),
              "ki_v": list(self.CP.longitudinalTuning.kiV),
              "dz_bp": list(self.CP.longitudinalTuning.deadzoneBP),
              "dz_v": list(self.CP.longitudinalTuning.deadzoneV)}
      json.dump(data, f)
    
  def reload_tuner(self):
    if not os.path.exists(self.tuner_filepath):
      print("LongControlTuner not ready")
      return
    
    modified_time = os.path.getmtime(self.tuner_filepath)
    # only if file modified
    if self.tuner_modified_time != modified_time:
      with open(self.tuner_filepath, 'r') as f:
        # read from tuner file
        try:
          data = json.load(f)
        except json.JSONDecodeError:
          data = {}
        if "kp_bp" in data and "kp_v" in data and "ki_bp" in data and "ki_v" in data and "dz_bp" in data and "dz_v" in data:
          self.pid = PIDController((data["kp_bp"], data["kp_v"]), 
                                   (data["ki_bp"], data["ki_v"]), 
                                   k_f=self.k_f, rate=1 / DT_CTRL)
          self.deadzoneBP = data["dz_bp"]
          self.deadzoneV = data["dz_v"]
        
          self.tuner_modified_time = modified_time
          print("LongControlTuner update:", json.dumps(data))
        else:
          print("LongControlTuner parsing failed")
          self.write_tuner()
          self.reload_tuner()

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    if self.pid:
      self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    """更新控制器状态和计算控制输出  
    主要步骤：
    1. 检查参数更新
    2. 计算目标速度和加速度
    3. 更新控制状态
    4. 执行PID控制
    """
    # 检查参数更新
    # check update
    current_time = time.time()
    if abs(current_time - self.tuner_last_check_update) >= self.tuner_update_interval:
      self.reload_tuner()
      self.tuner_last_check_update = current_time
    
    # 计算控制目标
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      # 插值计算当前目标速度和加速度
      v_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], long_plan.accels)

      # 考虑执行器延迟的上下界
      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      # 选择更保守的目标
      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, ModelConstants.T_IDXS[:CONTROL_N], speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      # 执行停车逻辑
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      # 执行PID控制
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    # 限制输出范围
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
