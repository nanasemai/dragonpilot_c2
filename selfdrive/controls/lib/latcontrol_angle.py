import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# 转向角饱和阈值（度）
# 当实际转向角与期望转向角的差值超过此阈值时，认为控制器处于饱和状态
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

class LatControlAngle(LatControl):
    """基于角度的横向控制器

    这是一个简单的横向控制器，直接控制转向角度而不是转向力矩。
    主要用于不支持转向力矩控制的车型。
    """
    def __init__(self, CP, CI):
        """初始化控制器

        参数:
        - CP: 车辆参数
        - CI: 车辆接口
        """
        super().__init__(CP, CI)
        # 饱和检查的最小速度阈值，低于此速度不进行饱和检查
        self.sat_check_min_speed = 5.

    def update(self, active, CS, VM, params, last_actuators, steer_limited,
              desired_curvature, desired_curvature_rate, llk):
        """更新控制器状态和输出

        参数:
        - active: 控制器是否激活
        - CS: 车辆状态
        - VM: 车辆模型
        - params: 控制参数
        - last_actuators: 上一次的执行器输出
        - steer_limited: 转向是否受限
        - desired_curvature: 期望曲率
        - desired_curvature_rate: 期望曲率变化率
        - llk: 车道线信息

        返回:
        - actuator: 执行器输出（此处始终为0，因为直接控制角度）
        - angle_steers_des: 期望转向角度
        - angle_log: 角度控制状态日志
        """
        # 创建角度控制状态日志
        angle_log = log.ControlsState.LateralAngleState.new_message()

        if not active:
            # 控制器未激活时，使用当前方向盘角度作为目标角度
            angle_log.active = False
            angle_steers_des = float(CS.steeringAngleDeg)
        else:
            # 控制器激活时，根据期望曲率计算目标转向角
            angle_log.active = True
            # 将期望曲率转换为转向角度
            # 注意曲率取负值是因为左转为正曲率，但左转为负角度
            angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
            # 添加角度偏移补偿
            angle_steers_des += params.angleOffsetDeg

        # 检查控制器是否饱和
        angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
        angle_log.saturated = self._check_saturation(angle_control_saturated, CS, False)

        # 记录实际和期望的转向角度
        angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
        angle_log.steeringAngleDesiredDeg = angle_steers_des

        # 返回控制输出和日志
        # 由于这是角度控制器，actuator输出为0，真正的控制通过angle_steers_des实现
        return 0, float(angle_steers_des), angle_log
