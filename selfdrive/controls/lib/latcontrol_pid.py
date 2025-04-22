import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter

class LatControlPID(LatControl):
    """基于PID的横向控制器

    特点：
    1. 使用经典PID控制算法
    2. 支持速度自适应的PID参数
    3. 包含前馈控制补偿
    4. 具有防饱和和重置机制
    5. 增加自适应增益调整
    6. 添加死区和平滑处理
    """
    def __init__(self, CP, CI):
        """初始化PID控制器

        参数:
        - CP: 车辆参数配置
        - CI: 车辆接口

        配置说明：
        - kpBP/kpV: 比例增益的速度断点和对应值
        - kiBP/kiV: 积分增益的速度断点和对应值
        - kf: 前馈增益
        """
        super().__init__(CP, CI)
        
        # 创建PID控制器实例
        self.pid = PIDController(
            (CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),  # 比例项参数
            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),  # 积分项参数
            k_f=CP.lateralTuning.pid.kf,                            # 前馈增益
            pos_limit=self.steer_max,                               # 正向限制
            neg_limit=-self.steer_max                               # 负向限制
        )
        
        # 获取转向前馈计算函数
        self.get_steer_feedforward = CI.get_steer_feedforward_function()
        
        # 添加自适应增益调整参数
        self.adaptive_gain = {
            'kp_min': 0.8,  # 最小比例增益系数
            'kp_max': 1.2,  # 最大比例增益系数
            'ki_min': 0.8,  # 最小积分增益系数
            'ki_max': 1.2,  # 最大积分增益系数
        }
        
        # 添加死区和平滑处理
        self.deadzone = 0.5  # 转向角死区(度)
        self.output_filter = FirstOrderFilter(0.0, 0.1, DT_MDL)  # 输出滤波器

    def _calculate_adaptive_gain(self, v_ego, steering_angle):
        """计算自适应增益系数
        
        基于车速和转向角度动态调整控制器增益
        - 高速时降低增益以提高稳定性
        - 大转向角时提高增益以提升响应性
        """
        # 车速影响因子(高速降低增益)
        speed_factor = np.interp(v_ego, 
                                [0, 10, 20, 30], 
                                [1.2, 1.0, 0.9, 0.8])
        
        # 转向角影响因子(大角度提高增益)
        angle_factor = np.interp(steering_angle,
                                [0, 45, 90],
                                [1.0, 1.1, 1.2])
                                
        return np.clip(speed_factor * angle_factor,
                      self.adaptive_gain['kp_min'],
                      self.adaptive_gain['kp_max'])

    def update(self, active, CS, VM, params, last_actuators, steer_limited,
               desired_curvature, desired_curvature_rate, llk):
        """更新控制器状态和计算控制输出

        主要步骤：
        1. 计算期望转向角
        2. 计算控制误差
        3. 计算前馈控制量
        4. 更新PID控制器
        5. 生成最终控制输出
        """
        # 创建PID状态日志
        pid_log = log.ControlsState.LateralPIDState.new_message()
        pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)    # 当前转向角
        pid_log.steeringRateDeg = float(CS.steeringRateDeg)      # 当前转向角速度

        # 计算期望转向角（不含偏移）
        angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(
            -desired_curvature, CS.vEgo, params.roll))

        # 添加角度偏移补偿
        angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

        # 计算控制误差
        error = angle_steers_des - CS.steeringAngleDeg
        
        # 当前死区处理
        if abs(error) < self.deadzone:
            error = 0.0
        # 建议增加渐变处理
        elif abs(error) < self.deadzone * 1.5:  # 死区过渡带
            error *= (abs(error) - self.deadzone) / (self.deadzone * 0.5)

        if not active:
            # 控制器未激活时重置状态
            output_steer = 0.0
            pid_log.active = False
            self.pid.reset()
        else:
            # 计算自适应增益
            kp_factor = self._calculate_adaptive_gain(CS.vEgo, abs(CS.steeringAngleDeg))
            ki_factor = self._calculate_adaptive_gain(CS.vEgo, abs(CS.steeringAngleDeg))
            
            # 更新PID控制器增益
            self.pid.adjust_gains(kp_factor, ki_factor)
            
            # 优化前馈控制计算
            ff_rate_factor = abs(CS.steeringRateDeg) * 0.05  # 转向速率影响因子
            ff_angle_factor = abs(angle_steers_des) * 0.01   # 目标转向角影响因子
            
            # 计算前馈控制量（不考虑偏移量的影响）
            steer_feedforward = self.get_steer_feedforward(
                angle_steers_des_no_offset, CS.vEgo) * (1.0 + ff_rate_factor + ff_angle_factor)

            # 更新PID控制器
            output_steer = self.pid.update(
                error,                          # 控制误差
                override=CS.steeringPressed,    # 驾驶员介入标志
                feedforward=steer_feedforward,  # 前馈控制量
                speed=CS.vEgo                   # 当前车速
            )
            
            # 对输出进行平滑处理
            output_steer = self.output_filter.update(output_steer)

            # 记录控制状态
            pid_log.active = True
            pid_log.p = self.pid.p             # 比例项输出
            pid_log.i = self.pid.i             # 积分项输出
            pid_log.f = self.pid.f             # 前馈项输出
            pid_log.output = output_steer      # 总输出

            # 检查控制器是否饱和
            pid_log.saturated = self._check_saturation(
                self.steer_max - abs(output_steer) < 1e-3,
                CS,
                steer_limited
            )

        return output_steer, angle_steers_des, pid_log