import math
import numpy as np

from common.numpy_fast import clip
from common.realtime import DT_CTRL
from cereal import log
from selfdrive.controls.lib.latcontrol import LatControl


class LatControlLQR(LatControl):
    """基于LQR(线性二次型调节器)的横向控制器

    LQR控制器的主要特点：
    1. 使用线性状态空间模型
    2. 包含卡尔曼状态观测器
    3. 带有积分器的反馈控制
    4. 速度自适应的转向力矩缩放
    """
    def __init__(self, CP, CI):
        """初始化控制器参数和状态空间矩阵

        状态空间模型参数：
        - A: 状态矩阵
        - B: 输入矩阵
        - C: 输出矩阵
        - K: LQR反馈增益矩阵
        - L: 卡尔曼增益矩阵
        """
        super().__init__(CP, CI)
        # 控制器基本参数
        self.scale = CP.lateralTuning.lqr.scale  # 控制输出缩放因子
        self.ki = CP.lateralTuning.lqr.ki        # 积分器增益

        # 状态空间模型矩阵
        self.A = np.array(CP.lateralTuning.lqr.a).reshape((2, 2))  # 状态矩阵
        self.B = np.array(CP.lateralTuning.lqr.b).reshape((2, 1))  # 输入矩阵
        self.C = np.array(CP.lateralTuning.lqr.c).reshape((1, 2))  # 输出矩阵
        self.K = np.array(CP.lateralTuning.lqr.k).reshape((1, 2))  # 反馈增益
        self.L = np.array(CP.lateralTuning.lqr.l).reshape((2, 1))  # 观测器增益
        self.dc_gain = CP.lateralTuning.lqr.dcGain  # 直流增益

        # 状态估计器初始化
        self.x_hat = np.array([[0], [0]])  # 状态估计值

        # 积分器参数
        self.i_unwind_rate = 0.3 * DT_CTRL  # 积分解缠绕率
        self.i_rate = 1.0 * DT_CTRL         # 积分更新率

    def update(self, active, CS, VM, params, last_actuators, steer_limited,
              desired_curvature, desired_curvature_rate, llk):
        """更新控制器状态和计算控制输出

        控制流程：
        1. 计算转向力矩缩放因子
        2. 更新状态估计
        3. 计算LQR控制输出
        4. 更新积分器
        5. 合成最终控制输出
        """
        # 创建日志对象
        lqr_log = log.ControlsState.LateralLQRState.new_message()

        # 速度自适应的转向力矩缩放
        torque_scale = (0.45 + CS.vEgo / 60.0)**2

        # 计算无偏转向角
        steering_angle_no_offset = CS.steeringAngleDeg - params.angleOffsetAverageDeg

        # 计算期望转向角
        desired_angle = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
        instant_offset = params.angleOffsetDeg - params.angleOffsetAverageDeg
        desired_angle += instant_offset

        # 更新卡尔曼滤波器状态估计
        angle_steers_k = float(self.C.dot(self.x_hat))
        e = steering_angle_no_offset - angle_steers_k
        self.x_hat = self.A.dot(self.x_hat) + self.B.dot(CS.steeringTorqueEps / torque_scale) + self.L.dot(e)

        if active:
            # 计算LQR基础控制量
            u_lqr = float(desired_angle / self.dc_gain - self.K.dot(self.x_hat))
            lqr_output = torque_scale * u_lqr / self.scale

            # 积分器更新
            if not CS.steeringPressed:
                # 计算积分误差
                error = desired_angle - angle_steers_k
                i = self.i_lqr + self.ki * self.i_rate * error
                control = lqr_output + i

                # 防饱和积分更新
                if (error >= 0 and (control <= self.steer_max or i < 0.0)) or \
                   (error <= 0 and (control >= -self.steer_max or i > 0.0)):
                    self.i_lqr = i
            else:
                # 驾驶员介入时解缠绕积分器
                self.i_lqr -= self.i_unwind_rate * float(np.sign(self.i_lqr))

            # 合成最终控制输出
            output_steer = lqr_output + self.i_lqr
            output_steer = clip(output_steer, -self.steer_max, self.steer_max)
        else:
            # 控制器未激活时重置状态
            lqr_output = 0.
            output_steer = 0.
            self.reset()

        return output_steer, desired_angle, lqr_log
