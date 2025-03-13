from collections import deque
import math
import numpy as np
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.car.interfaces import LatControlInputs
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.legacy_modeld.constants import ModelConstants
from openpilot.common.swaglog import cloudlog

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NN = [12, 3, 1, 0]
LAT_PLAN_MIN_IDX = 5
# 添加 NNFF 日志路径配置
#NNFF_LOG_DIR = "/data/media/0/c2_logs/nnff_log"

def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # compute finite difference between subsequent model_data.acceleration.y values
  # this is just two calls of np.diff followed by an element-wise division
  lat_accel_diffs = np.diff(lat_accels)
  lat_jerk = lat_accel_diffs / t_diffs
  # return as python list
  return lat_jerk.tolist()

def sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

class LatControlTorque(LatControl):
  """基于转矩的横向控制器

    主要特点：
    1. 直接控制转向力矩而不是角度
    2. 支持神经网络前馈控制(NNFF)
    3. 考虑低速补偿和摩擦力补偿
    4. 动态调整控制参数
  """
  def __init__(self, CP, CI):
    """初始化控制器

        参数配置：
        - torque_params: 转矩控制参数
        - use_nnff: 是否使用神经网络前馈
        - use_steering_angle: 是否使用方向盘角度作为反馈
     """
    super().__init__(CP, CI)
    # 基础控制参数
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.param_s = Params()
    self.torqued_override = self.param_s.get_bool("dp_torqued_override")
    self._frame = 0
    # dynamic steerActuatorDelay # 转向系统配置
    self.CP = CP
    self.enable_DSAD = False
    self.eps_torque_error = 0.0
    self.dsad = 0.0
    # 神经网络前馈(NNFF)配置
    # Twilsonco's Lateral Neural Network Feedforward
    self.use_nnff = CI.use_nnff
    self.use_nnff_lite = CI.use_nnff_lite
    if self.use_nnff or self.use_nnff_lite:
      #cloudlog.info(f"NNFF 初始化 - 模式: {'完整版' if self.use_nnff else '轻量版'},"
      #              f" 车型: {CP.carFingerprint}", log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
      # Instantaneous lateral jerk changes very rapidly, making it not useful on its own,
      # however, we can "look ahead" to the future planned lateral jerk in order to guage
      # whether the current desired lateral jerk will persist into the future, i.e.
      # whether it's "deliberate" or not. This lets us simply ignore short-lived jerk.
      # Note that LAT_PLAN_MIN_IDX is defined above and is used in order to prevent
      # using a "future" value that is actually planned to occur before the "current" desired
      # value, which is offset by the steerActuatorDelay.
      # 抖动控制参数
      self.friction_look_ahead_v = [1.4, 2.0] # how many seconds in the future to look ahead in [0, ~2.1] in 0.1 increments
      # 前瞻时间(秒)
      self.friction_look_ahead_bp = [9.0, 30.0] # corresponding speeds in m/s in [0, ~40] in 1.0 increments
      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      # 横向抖动摩擦系数
      self.lat_jerk_friction_factor = 0.4
      # 横向加速度摩擦系数
      self.lat_accel_friction_factor = 0.7 # in [0, 3], in 0.05 increments. 3 is arbitrary safety limit
      # precompute time differences between ModelConstants.T_IDXS
      # 时间序列配置
      self.t_diffs = np.diff(ModelConstants.T_IDXS)
      self.desired_lat_jerk_time = CP.steerActuatorDelay + 0.3

    if self.use_nnff:
      self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
      # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      self.nn_friction_override = CI.lat_torque_nn_model.friction_override

      # setup future time offsets
      self.nn_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.error_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update_live_tune(self):
    if self.enable_DSAD:
      self.desired_lat_jerk_time = self.dsad + 0.3
      # setup future time offsets
      self.nn_time_offset = self.dsad + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

    self._frame += 1
    if self._frame % 250 == 0:
      self._frame = 0
      self.torqued_override = self.param_s.get_bool("dp_torqued_override")
      if not self.torqued_override:
        return
      self.torque_params.latAccelFactor = float(self.param_s.get("dp_torque_lat_accel_factor", encoding="utf8")) * 0.01 # 1~500 delvalue=250
      self.torque_params.friction = float(self.param_s.get("dp_torque_friction", encoding="utf8")) * 0.01 #1~50 delvalue=1

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, model_data=None):
    """更新控制器状态和计算控制输出

        主要步骤：
        1. 计算实际和期望的横向加速度
        2. 应用低速补偿
        3. 计算NNFF控制量（如果启用）
        4. 更新PID控制器
        5. 生成最终控制输出
    """
    self.update_live_tune()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    #nn_log = None

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # 计算实际曲率和横向加速度
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      actual_lateral_jerk = 0.0
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        if self.use_nnff or self.use_nnff_lite:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      # 计算期望的横向加速度
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      # 低速补偿
      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y if not self.use_nnff else LOW_SPEED_Y_NN)**2
      # 计算控制目标和测量值
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature

      lateral_jerk_setpoint = 0
      lateral_jerk_measurement = 0
      lookahead_lateral_jerk = 0

      #是否使用NNFF
      model_good = model_data is not None and len(model_data.orientation.x) >= CONTROL_N \
                   and len(model_data.acceleration.y) >= len(ModelConstants.T_IDXS)
      if model_good and (self.use_nnff or self.use_nnff_lite):
        # # 每 100 帧记录一次基础状态
        # if self._frame % 100 == 0:
        #  cloudlog.debug(f"NNFF 状态 - 速度: {CS.vEgo:.1f}m/s, 横向加速度: {actual_lateral_accel:.2f}m/s², 期望加速度: {desired_lateral_accel:.2f}m/s²",
        #               log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
        # # 当横向加速度变化较大时记录
        # if abs(desired_lateral_accel - actual_lateral_accel) > 1.0:
        #  cloudlog.info(f"NNFF 大幅变化 - 横向加速度差值: {desired_lateral_accel - actual_lateral_accel:.2f}m/s²",
        #                  log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
        # # 记录抖动控制相关信息（当抖动值较大时）
        #  if abs(lookahead_lateral_jerk) > 2.0:
        #    cloudlog.debug(f"NNFF 抖动控制 - 前瞻抖动: {lookahead_lateral_jerk:.2f}, 实际抖动: {actual_lateral_jerk:.2f}",
        #                   log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
        # prepare "look-ahead" desired lateral jerk
        lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
        desired_lateral_jerk = (interp(self.desired_lat_jerk_time, ModelConstants.T_IDXS, model_data.acceleration.y) - desired_lateral_accel) / self.desired_lat_jerk_time
        lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk)
        if self.use_steering_angle or lookahead_lateral_jerk == 0.0:
          lookahead_lateral_jerk = 0.0
          actual_lateral_jerk = 0.0
          self.lat_accel_friction_factor = 1.0
        lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
        lateral_jerk_measurement = self.lat_jerk_friction_factor * actual_lateral_jerk

      if self.use_nnff and model_good:
        # update past data
        pitch = 0
        roll = params.roll
        if len(llk.calibratedOrientationNED.value) > 1:
          pitch = self.pitch.update(llk.calibratedOrientationNED.value[1])
          roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(desired_lateral_accel)
        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nn_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [roll_pitch_adjust(interp(t, ModelConstants.T_IDXS, model_data.orientation.x) + roll, interp(t, ModelConstants.T_IDXS, model_data.orientation.y) + pitch) for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [interp(t, ModelConstants.T_IDXS[:CONTROL_N], model_data.acceleration.y) for t in adjusted_future_times]

        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                              + [setpoint] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                                 + [measurement] * self.past_future_len \
                                 + past_rolls + future_rolls
        torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
        torque_from_measurement = self.torque_from_nn(nnff_measurement_input)

        pid_log.error = torque_from_setpoint - torque_from_measurement
        error_blend_factor = interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0])

        if error_blend_factor > 0.0:  # blend in stronger error response when in high lat accel
          nnff_error_input = [CS.vEgo, setpoint - measurement, lateral_jerk_setpoint - lateral_jerk_measurement, 0.0]
          torque_from_error = self.torque_from_nn(nnff_error_input)
          if sign(pid_log.error) == sign(torque_from_error) and abs(pid_log.error) < abs(torque_from_error):
            pid_log.error = pid_log.error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor

        # compute feedforward (same as nn setpoint output)
        error = setpoint - measurement
        friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        # 使用神经网络计算前馈控制量
        nn_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
                   + past_lateral_accels_desired + future_planned_lateral_accels \
                   + past_rolls + future_rolls
        ff = self.torque_from_nn(nn_input)
        # apply friction override for cars with low NN friction response

        if self.nn_friction_override:
          pid_log.error += self.torque_from_lateral_accel(LatControlInputs(0.0, 0.0, CS.vEgo, CS.aEgo), self.torque_params,
                                                          friction_input, lateral_accel_deadzone, friction_compensation=True)
        #nn_log = nn_input + nnff_setpoint_input + nnff_measurement_input
        # # 记录神经网络输出（当输出较大时）
        # if abs(torque_from_setpoint) > 1.0:
        #   cloudlog.debug(f"NNFF 网络输出 - 设定值: {torque_from_setpoint:.2f}, 测量值: {torque_from_measurement:.2f}, 误差: {pid_log.error:.2f}",
        #     log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
      else:# 不使用NNFF或者模型不对
        gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
        torque_from_setpoint = self.torque_from_lateral_accel(LatControlInputs(setpoint, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                              lateral_jerk_setpoint, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite)
        torque_from_measurement = self.torque_from_lateral_accel(LatControlInputs(measurement, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                                                 lateral_jerk_measurement, lateral_accel_deadzone, friction_compensation=self.use_nnff_lite)
        pid_log.error = torque_from_setpoint - torque_from_measurement
        error = desired_lateral_accel - actual_lateral_accel
        if self.use_nnff_lite:
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        else:
          friction_input = error
        # 使用传统方法计算前馈控制量
        ff = self.torque_from_lateral_accel(LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo), self.torque_params,
                                            friction_input, lateral_accel_deadzone, friction_compensation=True)

      # 更新PID控制器
      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)
      #if nn_log is not None:
      #  pid_log.nnLog = nn_log

    # # 记录最终控制输出（当力矩较大时）
    # if abs(output_torque) > 2.0:
    #   cloudlog.debug(f"NNFF 控制输出 - 力矩: {output_torque:.2f}, P: {self.pid.p:.2f}, I: {self.pid.i:.2f}, D: {self.pid.d:.2f}, F: {self.pid.f:.2f}",
    #     log_dir=NNFF_LOG_DIR, module_name="NNFF-INFO")
    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
