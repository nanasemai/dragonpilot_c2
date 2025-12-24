# MIT Non-Commercial License
#
# Copyright (c) 2019, dragonpilot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:
#
# - The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# - Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
# - Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Downhill Coasting allows the vehicle to maintain or slightly increase speed on downhill slopes without braking.
import numpy as np
from common.params import Params
# 坡度阈值，当sin(pitch)小于此值时判定为下坡
SLOPE = -0.04
# 车速比例阈值，当前车速超过巡航速度*RATIO时判定为超速
RATIO = 0.9
# 前车碰撞时间(TTC)相关参数
TTC = 5.  # 前车碰撞时间阈值(秒)
TTC_BP = [2.5, 3.5]  # 碰撞时间插值点 (修正为单调递增)
MIN_BRAKE_ALLOW_VALS = [-0.5, 0.0]  # 对应不同碰撞时间允许的最小刹车值 (对应修正)

# 安全限制参数
MAX_SPEED_OVER_CRUISE = 5.0  # 允许超过巡航速度的最大值 (m/s)
SPEED_HYSTERESIS = 1.0       # 车速迟滞值，避免状态频繁切换 (m/s)
MIN_ACTIVE_SPEED = 5.0       # 最低激活速度 (m/s)
MIN_SAFE_DISTANCE = 30.0     # 最小安全距离 (m)

def get_min_accel():
    """安全地获取最小加速度参数，带异常处理"""
    try:
        val = Params().get("dp_lon_acm_min_accel", encoding="utf8")
        return np.clip(float(val) * 0.01, -0.5, 0.5) if val else -0.1
    except (TypeError, ValueError):
        return -0.1

class ACM:
  def __init__(self, enabled = False, downhill_only = False):
    self._enabled = enabled  # ACM功能是否启用
    self._downhill_only = downhill_only  # 是否仅在下坡时启用ACM
    self._is_downhill = False  # 当前是否处于下坡状态
    self._is_speed_over_cruise = False  # 当前车速是否超过巡航速度的RATIO比例
    self._has_lead = False  # 前方是否有前车
    self._active_prev = False  # 上一循环ACM是否激活
    self.will_activate_on_gas_release = False # 新增状态：预判松油门后是否会激活

    self.active = False  # ACM当前是否激活
    self.just_disabled = False  # ACM是否刚刚被禁用
    self.allowed_brake_val = 0.  # 允许的最小刹车值
    self.lead_ttc = float('inf')  # 与前车的碰撞时间(Time To Collision)，默认无穷大
    self.lead_distance = float('inf')  # 与前车距离
    self.v_ego = 0.0  # 当前车速
    self.v_cruise = 0.0  # 巡航速度
    self._min_accel = get_min_accel()  # 在初始化时读取一次参数
    
    # 参数读取频率控制
    self._params_counter = 0
    self._PARAMS_UPDATE_INTERVAL = 100  # 100帧更新一次（假设100Hz循环，即1秒更新一次）

  def update_states(self, cs, rs, user_ctrl_lon, v_ego, v_cruise):
    """更新ACM状态
    参数:
    - cs: 车辆状态
    - rs: 雷达状态
    - user_ctrl_lon: 用户是否在控制纵向
    - v_ego: 当前车速
    - v_cruise: 巡航速度
    """
    self.lead_ttc = float('inf')  # 如果没有前车，默认为无穷大
    self.v_ego = v_ego # 保存当前车速
    self.v_cruise = v_cruise # 保存巡航速度
    
    # 动态更新最小加速度参数（限制频率）
    self._params_counter += 1
    if self._params_counter >= self._PARAMS_UPDATE_INTERVAL:
      self._min_accel = get_min_accel()
      self._params_counter = 0

    # 如果ACM功能未启用，则直接返回，不激活ACM
    if not self._enabled:
      self.active = False
      return

    # 如果车辆姿态数据不完整，则不激活ACM
    if len(cs.orientationNED) != 3:
      self.active = False
      return

    # 计算当前是否处于下坡状态
    pitch_rad = cs.orientationNED[1]  # 获取俯仰角（弧度）
    self._is_downhill = np.sin(pitch_rad) < SLOPE # 判断是否为下坡
    # 计算当前车速是否在安全范围内（带迟滞机制）
    speed_above_threshold = v_ego > (v_cruise * RATIO)
    
    # 引入迟滞：当ACM当前激活时，使用更高的上限阈值；否则使用正常阈值
    if self.active:
        speed_upper_limit = v_cruise + MAX_SPEED_OVER_CRUISE + SPEED_HYSTERESIS
    else:
        speed_upper_limit = v_cruise + MAX_SPEED_OVER_CRUISE
    
    speed_below_limit = v_ego < speed_upper_limit
    speed_above_minimum = v_ego > MIN_ACTIVE_SPEED
    self._is_speed_in_range = speed_above_threshold and speed_below_limit and speed_above_minimum

    # 获取前车信息
    lead = rs.leadOne
    if lead and lead.status and v_ego > 0.5: # 检查前车是否存在且有效，并且当前车速大于0.5m/s
      self.lead_distance = lead.dRel  # 保存距离
      
      # 正确的TTC计算：使用相对速度而非自车速度
      # 使用更保守的阈值(-0.3)避免在接近等速时的频繁跳变
      if lead.vRel < -0.3:  # 只有在明显接近前车时才有意义 (vRel < 0 表示接近)
        self.lead_ttc = max(0.1, -lead.dRel / lead.vRel)  # 避免除零和负值，确保TTC为正
      else:
        self.lead_ttc = float('inf')  # 远离前车或无碰撞风险
      
      # 根据TTC插值计算允许的最小刹车值
      self.allowed_brake_val = np.interp(self.lead_ttc, TTC_BP, MIN_BRAKE_ALLOW_VALS)
      
      # 增加距离条件：TTC危险或距离过近都视为有前车
      ttc_danger = self.lead_ttc < TTC
      distance_danger = self.lead_distance < MIN_SAFE_DISTANCE
      self._has_lead = ttc_danger or distance_danger
    else:
      self._has_lead = False # 无有效前车
      self.lead_ttc = float('inf') # 无前车时TTC为无穷大
      self.lead_distance = float('inf') # 无前车时距离为无穷大
      # 当没有前车时，允许的刹车值应对应于高TTC（即0.0刹车）
      self.allowed_brake_val = MIN_BRAKE_ALLOW_VALS[-1] # 或者使用插值 np.interp(self.lead_ttc, TTC_BP, MIN_BRAKE_ALLOW_VALS)

    # 更新ACM激活状态：
    # 1. 用户未控制纵向
    # 2. 无近距离前车
    # 3. 当前车速在安全范围内（超过巡航速度阈值但不超过上限，且高于最低激活速度）
    # 4. 如果设置了仅下坡模式，则必须处于下坡状态；否则平路也可以抑制刹车
    self.active = not user_ctrl_lon and not self._has_lead and self._is_speed_in_range and (self._is_downhill if self._downhill_only else True)

    # 计算预激活条件：
    # 当用户正在控制纵向（踩油门），但其他ACM激活条件（无前车、速度范围、坡度条件）均已满足时，
    # 则预判松开油门后ACM会激活。
    conditions_met_except_gas = not self._has_lead and self._is_speed_in_range and (self._is_downhill if self._downhill_only else True)
    self.will_activate_on_gas_release = user_ctrl_lon and conditions_met_except_gas

    # 判断ACM是否刚刚被禁用
    self.just_disabled = self._active_prev and not self.active
    # 更新上一循环的ACM激活状态
    self._active_prev = self.active

  def update_a_desired_trajectory(self, a_desired_trajectory):
    """更新期望加速度轨迹
    如果ACM激活，则根据允许的刹车值调整轨迹中的减速度
    """
    # 如果ACM未激活，则直接返回原始轨迹
    if not self.active:
      return a_desired_trajectory

    # 遍历期望加速度轨迹的每个点
    for i in range(len(a_desired_trajectory)):
      accel_val = a_desired_trajectory[i] # 获取当前点的加速度值
      if accel_val < 0:  # 如果系统希望在该轨迹点刹车
        # 简化的刹车限制逻辑：取系统请求、允许值和最小加速度中的最大值
        # 这样可以确保刹车力度不会超过允许的限制，同时保持平滑滑行
        a_desired_trajectory[i] = max(accel_val, self.allowed_brake_val, self._min_accel)
    return a_desired_trajectory

  def update_output_a_target(self, output_a_target):
    """更新输出加速度目标
    抑制不必要的刹车以允许平滑滑行
    返回处理后的加速度目标
    """
    # 优先级1: ACM已激活状态（最高优先级）
    if self.active and output_a_target < 0:
      # ACM激活时，根据前车约束限制刹车力度
      return max(output_a_target, self.allowed_brake_val, self._min_accel)
    
    # 优先级2: 预判松油门后激活ACM（次优先级）
    if self.will_activate_on_gas_release and output_a_target < 0:
      # 预判纯滑行模式，统一使用相同的逻辑
      return max(output_a_target, self.allowed_brake_val, self._min_accel)
    
    # 其他情况：返回原始加速度目标
    return output_a_target

  def set_enabled(self, enabled):
    """设置ACM功能是否启用"""
    self._enabled = enabled

  def set_downhill_only(self, downhill_only):
    """设置是否仅在下坡时启用ACM"""
    self._downhill_only = downhill_only
