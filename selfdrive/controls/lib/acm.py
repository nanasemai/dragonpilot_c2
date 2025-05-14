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
# 坡度阈值，当sin(pitch)小于此值时判定为下坡
SLOPE = -0.04
# 车速比例阈值，当前车速超过巡航速度*RATIO时判定为超速
RATIO = 0.9
# 前车碰撞时间(TTC)相关参数
TTC = 5.  # 前车碰撞时间阈值(秒)
TTC_BP = [5.0, 3.0]  # 碰撞时间插值点
MIN_BRAKE_ALLOW_VALS = [0., -0.5]  # 对应不同碰撞时间允许的最小刹车值

class ACM:
  def __init__(self, enabled = False, downhill_only = False):
    self._enabled = enabled  # ACM功能是否启用
    self._downhill_only = downhill_only  # 是否仅在下坡时启用ACM
    self._is_downhill = False  # 当前是否处于下坡状态
    self._is_speed_over_cruise = False  # 当前车速是否超过巡航速度的RATIO比例
    self._has_lead = False  # 前方是否有前车
    self._active_prev = False  # 上一循环ACM是否激活

    self.active = False  # ACM当前是否激活
    self.just_disabled = False  # ACM是否刚刚被禁用
    self.allowed_brake_val = 0.  # 允许的最小刹车值
    self.lead_ttc = float('inf')  # 与前车的碰撞时间(Time To Collision)，默认无穷大

  def update_states(self, cs, rs, user_ctrl_lon, v_ego, v_cruise):
    """更新ACM状态
    参数:
    - cs: 车辆状态
    - rs: 雷达状态
    - user_ctrl_lon: 用户是否在控制纵向
    - v_ego: 当前车速
    - v_cruise: 巡航速度
    """
    self.lead_ttc = float('inf')  # Default if no lead

    if not self._enabled:
      self.active = False
      return

    if len(cs.orientationNED) != 3:
      self.active = False
      return

    pitch_rad = cs.orientationNED[1]
    self._is_downhill = np.sin(pitch_rad) < SLOPE
    self._is_speed_over_cruise = v_ego > (v_cruise * RATIO)

    lead = rs.leadOne
    if lead and lead.status:
      self.lead_ttc = lead.dRel / v_ego if v_ego > 0 else float('inf')
      self.allowed_brake_val = np.interp(self.lead_ttc, TTC_BP, MIN_BRAKE_ALLOW_VALS)
      self._has_lead = self.lead_ttc < TTC
    else:
      self._has_lead = False

    self.active = not user_ctrl_lon and not self._has_lead and self._is_speed_over_cruise and (self._is_downhill if self._downhill_only else True)

    self.just_disabled = self._active_prev and not self.active
    self._active_prev = self.active

  def update_a_desired_trajectory(self, a_desired_trajectory):
    if not self.active:
      return a_desired_trajectory

    # Suppress all braking to allow smooth coasting
    for i in range(len(a_desired_trajectory)):
      if a_desired_trajectory[i] < 0 and a_desired_trajectory[i] > self.allowed_brake_val:
        a_desired_trajectory[i] = 0.0
    return a_desired_trajectory

  def update_output_a_target(self, output_a_target):
    """更新输出加速度目标
    抑制不必要的刹车以允许平滑滑行
    返回处理后的加速度目标
    """
    if not self.active:
      return output_a_target

    # Suppress braking
    if output_a_target < 0 and output_a_target > self.allowed_brake_val:
      output_a_target = 0.0
    return output_a_target

  def set_enabled(self, enabled):
    """设置ACM功能是否启用"""
    self._enabled = enabled

  def set_downhill_only(self, downhill_only):
    """设置是否仅在下坡时启用ACM"""
    self._downhill_only = downhill_only
