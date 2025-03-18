from abc import abstractmethod, ABC

from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL

MIN_LATERAL_CONTROL_SPEED = 0.3  # m/s


class LatControl(ABC):
    """横向控制器基类

    这是所有横向控制器的抽象基类，定义了基本的接口和通用功能：
    1. 饱和检测机制
    2. 控制器重置
    3. 标准化的转向力矩范围
    """
    def __init__(self, CP, CI):
        """初始化控制器基本参数

        参数:
        - CP: 车辆参数配置
        - CI: 车辆接口

        初始化内容：
        1. 饱和检测参数
        2. 转向限制参数
        3. 最小速度阈值
        """
        # 饱和计数相关参数
        self.sat_count_rate = 1.0 * DT_CTRL  # 饱和计数增长率
        self.sat_limit = CP.steerLimitTimer   # 饱和时间限制
        self.sat_count = 0.                   # 饱和计数器
        self.sat_check_min_speed = 10.        # 饱和检查最小速度(m/s)

        # 转向力矩范围标准化为[-1.0, 1.0]
        self.steer_max = 1.0

    @abstractmethod
    def update(self, active, CS, VM, params, last_actuators, steer_limited,
               desired_curvature, desired_curvature_rate, llk):
        """更新控制器状态和计算控制输出

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
        """
        pass

    def reset(self):
        """重置控制器状态

        主要重置饱和计数器
        """
        self.sat_count = 0.

    def _check_saturation(self, saturated, CS, steer_limited):
        """检查转向控制是否处于饱和状态

        条件：
        1. 当前输出达到饱和
        2. 车速高于最小阈值
        3. 转向未受限
        4. 驾驶员未介入

        参数:
        - saturated: 是否达到饱和
        - CS: 车辆状态
        - steer_limited: 转向是否受限

        返回:
        - bool: 是否处于饱和状态
        """
        # 满足饱和条件时增加计数
        if (saturated and
            CS.vEgo > self.sat_check_min_speed and
            not steer_limited and
            not CS.steeringPressed):
            self.sat_count += self.sat_count_rate
        else:
            # 不满足条件时减少计数
            self.sat_count -= self.sat_count_rate

        # 限制计数器范围
        self.sat_count = clip(self.sat_count, 0.0, self.sat_limit)

        # 判断是否达到饱和状态
        return self.sat_count > (self.sat_limit - 1e-3)
