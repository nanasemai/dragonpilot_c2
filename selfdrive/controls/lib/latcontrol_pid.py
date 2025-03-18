import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController


class LatControlPID(LatControl):
    """基于PID的横向控制器

    特点：
    1. 使用经典PID控制算法
    2. 支持速度自适应的PID参数
    3. 包含前馈控制补偿
    4. 具有防饱和和重置机制
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

        if not active:
            # 控制器未激活时重置状态
            output_steer = 0.0
            pid_log.active = False
            self.pid.reset()
        else:
            # 计算前馈控制量（不考虑偏移量的影响）
            steer_feedforward = self.get_steer_feedforward(
                angle_steers_des_no_offset, CS.vEgo)

            # 更新PID控制器
            output_steer = self.pid.update(
                error,                          # 控制误差
                override=CS.steeringPressed,    # 驾驶员介入标志
                feedforward=steer_feedforward,  # 前馈控制量
                speed=CS.vEgo                   # 当前车速
            )

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
