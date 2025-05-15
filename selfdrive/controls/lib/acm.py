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
# TTC_BP = [5.0, 3.0]  # 碰撞时间插值点 (旧)
# MIN_BRAKE_ALLOW_VALS = [0., -0.5]  # 对应不同碰撞时间允许的最小刹车值 (旧)
TTC_BP = [3.0, 5.0]  # 碰撞时间插值点 (修正为单调递增)
MIN_BRAKE_ALLOW_VALS = [-0.5, 0.0]  # 对应不同碰撞时间允许的最小刹车值 (对应修正)

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
    # 计算当前车速是否超过巡航速度的一定比例
    self._is_speed_over_cruise = v_ego > (v_cruise * RATIO)

    # 获取前车信息
    lead = rs.leadOne
    if lead and lead.status and v_ego > 0.1: # 检查前车是否存在且有效，并且当前车速大于0.1m/s以避免除零错误
      self.lead_ttc = lead.dRel / v_ego if v_ego > 0 else float('inf') # 计算与前车的碰撞时间 (TTC)
      # 根据TTC插值计算允许的最小刹车值
      self.allowed_brake_val = np.interp(self.lead_ttc, TTC_BP, MIN_BRAKE_ALLOW_VALS)
      self._has_lead = self.lead_ttc < TTC # 判断是否存在近距离前车
    else:
      self._has_lead = False # 无有效前车
      self.lead_ttc = float('inf') # 无前车时TTC为无穷大
      # 当没有前车时，允许的刹车值应对应于高TTC（即0.0刹车）
      self.allowed_brake_val = MIN_BRAKE_ALLOW_VALS[-1] # 或者使用插值 np.interp(self.lead_ttc, TTC_BP, MIN_BRAKE_ALLOW_VALS)

    # 更新ACM激活状态：
    # 1. 用户未控制纵向
    # 2. 无近距离前车
    # 3. 当前车速超过巡航速度的一定比例
    # 4. 如果设置了仅下坡模式，则必须处于下坡状态；否则此条件为真
    self.active = not user_ctrl_lon and not self._has_lead and self._is_speed_over_cruise and (self._is_downhill if self._downhill_only else True)

    # 计算预激活条件：
    # 当用户正在控制纵向（踩油门），但其他ACM激活条件（无前车、超速、满足下坡条件）均已满足时，
    # 则预判松开油门后ACM会激活。
    conditions_met_except_gas = not self._has_lead and self._is_speed_over_cruise and (self._is_downhill if self._downhill_only else True)
    self.will_activate_on_gas_release = bool(user_ctrl_lon and conditions_met_except_gas) # 确保结果是布尔值

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
        if self.allowed_brake_val == 0.0:  # 如果是纯滑行模式（无前车或前车距离远）
          a_desired_trajectory[i] = 0.0 # 将期望加速度设为0，实现滑行
        elif accel_val > self.allowed_brake_val:  # 如果系统请求的刹车力度比允许的最小刹车值更温和
          a_desired_trajectory[i] = 0.0  # 选择滑行，而不是轻微刹车
        else:  # 如果系统请求的刹车力度大于或等于允许的最小刹车值 (accel_val <= self.allowed_brake_val)
          a_desired_trajectory[i] = self.allowed_brake_val  # 将刹车力度限制在允许的最大值
    return a_desired_trajectory

  def update_output_a_target(self, output_a_target):
    """更新输出加速度目标
    抑制不必要的刹车以允许平滑滑行
    返回处理后的加速度目标
    """
    # 条件1: ACM已激活
    # 条件2: 预判到松油门后会激活ACM，并且当前允许的刹车值为0.0（即目标是纯滑行）
    # 并且系统当前的加速度目标是负值（即想要刹车）
    if (self.active or (self.will_activate_on_gas_release and self.allowed_brake_val == 0.0)) and output_a_target < 0:
      # 在纯滑行或预判纯滑行的情况下，直接将目标加速度设为0
      # 注意：如果预判时也需要考虑带前车的情况，这里的逻辑会更复杂，需要确保 allowed_brake_val 的正确性
      # 这种情况下，即使ACM尚未正式激活（因为用户可能还在踩油门），但由于预判到松油门后会进入纯滑行状态，
      # 所以提前将MPC等模块计算出的减速度抑制掉，避免松油门瞬间的顿挫感。
      output_a_target = 0.0
    elif self.active and output_a_target < 0: # ACM激活且有前车约束的情况 (self.allowed_brake_val 可能小于 0)
        # 这种情况下，ACM已经激活，并且可能存在前车，所以需要根据 allowed_brake_val 来限制刹车
        if output_a_target > self.allowed_brake_val:  # 如果系统请求的刹车力度比允许的最小刹车值更温和
            output_a_target = 0.0  # 选择滑行，而不是轻微刹车
        else:  # 如果系统请求的刹车力度大于或等于允许的最小刹车值 (output_a_target <= self.allowed_brake_val)
            output_a_target = self.allowed_brake_val  # 将刹车力度限制在允许的最大值
    return output_a_target

  def set_enabled(self, enabled):
    """设置ACM功能是否启用"""
    self._enabled = enabled

  def set_downhill_only(self, downhill_only):
    """设置是否仅在下坡时启用ACM"""
    self._downhill_only = downhill_only
