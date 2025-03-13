#!/usr/bin/env python3
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Version = 2024-9-28
from common.numpy_fast import interp
import time
import numpy as np
from openpilot.common.realtime import DT_MDL

# d-e2e, from modeldata.h
# 轨迹预测点数量
TRAJECTORY_SIZE = 33

# 前车检测参数
LEAD_WINDOW_SIZE = 3      # 前车检测滑动窗口大小
LEAD_PROB = 0.5          # 前车检测概率阈值，大于此值认为确实存在前车

# 减速场景检测参数
SLOW_DOWN_WINDOW_SIZE = 5  # 减速检测滑动窗口大小
SLOW_DOWN_PROB = 0.5      # 减速检测概率阈值，大于此值触发减速

# 减速距离参数配置 (速度单位：km/h，距离单位：米)
SLOW_DOWN_BP =   [0.,  10.,  20.,   30.,   40.,   50.,   55.,   60.]  # 速度断点
SLOW_DOWN_DIST = [20., 30.,  45.,   65.,   90.,   120.,  140.,  165.]  # 对应的安全距离

# 低速巡航检测参数
SLOWNESS_WINDOW_SIZE = 10   # 低速检测滑动窗口大小
SLOWNESS_PROB = 0.5        # 低速检测概率阈值
SLOWNESS_CRUISE_OFFSET = 1.05  # 巡航速度偏移系数，实际速度低于设定速度的105%认为是低速

# 危险时间间隔(TTC)检测参数
DANGEROUS_TTC_WINDOW_SIZE = 3  # TTC检测滑动窗口大小
DANGEROUS_TTC = 2.3           # 危险TTC阈值(秒)，小于此值认为存在碰撞风险

# 高速巡航速度阈值(km/h)
HIGHWAY_CRUISE_KPH = 70      # 高于此速度认为是高速巡航状态

# 停走检测参数
STOP_AND_GO_FRAME = 60       # 停走状态持续帧数

# 模式切换超时设置
SET_MODE_TIMEOUT = 10        # 模式切换超时帧数

# 前向碰撞预警(FCW)参数
MPC_FCW_WINDOW_SIZE = 10     # FCW检测滑动窗口大小
MPC_FCW_PROB = 0.5          # FCW触发概率阈值

# 最低巡航速度(m/s)，约35km/h
V_ACC_MIN = 9.72            # ACC模式最低工作速度

HIGHWAY_SPEED_MIN = 30.0  # 高速最低速度阈值(km/h)
MODE_SWITCH_HYSTERESIS = 2.0  # 模式切换滞后时间(秒)
DANGEROUS_TTC_HIGH_SPEED = 3.0  # 高速时的危险TTC阈值
DANGEROUS_TTC_LOW_SPEED = 2.0   # 低速时的危险TTC阈值

class SNG_State:
  off = 0
  stopped = 1
  going = 2


class GenericMovingAverageCalculator:
  def __init__(self, window_size):
    self.window_size = window_size
    self.data = []
    self.total = 0

  def add_data(self, value):
    if len(self.data) == self.window_size:
      self.total -= self.data.pop(0)
    self.data.append(value)
    self.total += value

  def get_moving_average(self):
    if len(self.data) == 0:
      return None
    return self.total / len(self.data)

  def reset_data(self):
    self.data = []
    self.total = 0


class DynamicEndtoEndController:

  def __init__(self):
    self._is_enabled = False
    self._mode = 'acc'
    self._mode_prev = 'acc'
    self._frame = 0

    self._lead_gmac = GenericMovingAverageCalculator(window_size=LEAD_WINDOW_SIZE)
    self._has_lead_filtered = False
    self._has_lead_filtered_prev = False

    self._slow_down_gmac = GenericMovingAverageCalculator(window_size=SLOW_DOWN_WINDOW_SIZE)
    self._has_slow_down = False

    self._has_blinkers = False

    self._slowness_gmac = GenericMovingAverageCalculator(window_size=SLOWNESS_WINDOW_SIZE)
    self._has_slowness = False

    self._has_nav_enabled = False

    self._dangerous_ttc_gmac = GenericMovingAverageCalculator(window_size=DANGEROUS_TTC_WINDOW_SIZE)
    self._has_dangerous_ttc = False

    self._v_ego_kph = 0.
    self._v_cruise_kph = 0.

    self._has_lead = False

    self._has_standstill = False
    self._has_standstill_prev = False

    self._sng_transit_frame = 0
    self._sng_state = SNG_State.off

    self._mpc_fcw_gmac = GenericMovingAverageCalculator(window_size=MPC_FCW_WINDOW_SIZE)
    self._has_mpc_fcw = False
    self._mpc_fcw_crash_cnt = 0

    self._set_mode_timeout = 0

    self._last_mode_switch = time.monotonic()
    self._last_lead_distance = 0.0
    self._lead_accel = 0.0
    self._last_ttc = float('inf')
    self._ttc_rate = 0.0
    pass


  def _update_lead_dynamics(self,car_state,lead_one, dt):
        if self._has_lead:
            distance_diff = lead_one.dRel - self._last_lead_distance
            self._lead_accel = distance_diff / (dt + 1e-6)
            self._last_lead_distance = lead_one.dRel

            # 计算TTC变化率
            current_ttc = lead_one.dRel / (car_state.vEgo + 1e-6)
            self._ttc_rate = (current_ttc - self._last_ttc) / dt
            self._last_ttc = current_ttc

  def _get_dynamic_ttc_threshold(self):
      base_threshold = interp(self._v_ego_kph,
                            [0., HIGHWAY_SPEED_MIN, HIGHWAY_CRUISE_KPH],
                            [DANGEROUS_TTC_LOW_SPEED, DANGEROUS_TTC_LOW_SPEED, DANGEROUS_TTC_HIGH_SPEED])

      # 根据前车加速度调整阈值
      accel_factor = 1.0 + np.clip(self._lead_accel * 0.1, -0.3, 0.3)
      # 根据TTC变化率调整阈值
      ttc_rate_factor = 1.0 - np.clip(self._ttc_rate * 0.2, -0.2, 0.2)

      return base_threshold * accel_factor * ttc_rate_factor

  def _check_mode_switch_conditions(self, current_time):
      if current_time - self._last_mode_switch < MODE_SWITCH_HYSTERESIS:
          return False
      return True


  def _update(self, car_state, lead_one, md, controls_state):
    current_time = time.monotonic()
    dt = DT_MDL  # 假设使用模型的时间步长

    # 更新前车动态特性
    self._update_lead_dynamics(car_state,lead_one, dt)

    self._v_ego_kph = car_state.vEgo * 3.6
    self._v_cruise_kph = controls_state.vCruise
    self._has_lead = lead_one.status
    self._has_standstill = car_state.standstill

    # fcw detection
    self._mpc_fcw_gmac.add_data(self._mpc_fcw_crash_cnt > 0)
    self._has_mpc_fcw = self._mpc_fcw_gmac.get_moving_average() > MPC_FCW_PROB

    # nav enable detection
    self._has_nav_enabled = md.navEnabled

    # lead detection
    self._lead_gmac.add_data(lead_one.status)
    self._has_lead_filtered = self._lead_gmac.get_moving_average() > LEAD_PROB

    # slow down detection
    self._slow_down_gmac.add_data(len(md.orientation.x) == len(md.position.x) == TRAJECTORY_SIZE and md.position.x[TRAJECTORY_SIZE - 1] < interp(self._v_ego_kph, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    self._has_slow_down = self._slow_down_gmac.get_moving_average() > SLOW_DOWN_PROB

    # blinker detection
    self._has_blinkers = car_state.leftBlinker or car_state.rightBlinker

    # sng detection
    if self._has_standstill:
      self._sng_state = SNG_State.stopped
      self._sng_transit_frame = 0
    else:
      if self._sng_transit_frame == 0:
        if self._sng_state == SNG_State.stopped:
          self._sng_state = SNG_State.going
          self._sng_transit_frame = STOP_AND_GO_FRAME
        elif self._sng_state == SNG_State.going:
          self._sng_state = SNG_State.off
      elif self._sng_transit_frame > 0:
        self._sng_transit_frame -= 1

    # slowness detection
    if not self._has_standstill:
      self._slowness_gmac.add_data(self._v_ego_kph <= (self._v_cruise_kph*SLOWNESS_CRUISE_OFFSET))
      self._has_slowness = self._slowness_gmac.get_moving_average() > SLOWNESS_PROB

    # dangerous TTC detection
    if not self._has_lead_filtered and self._has_lead_filtered_prev:
      self._dangerous_ttc_gmac.reset_data()
      self._has_dangerous_ttc = False

    # 使用动态TTC阈值进行危险判断
    if self._has_lead and car_state.vEgo >= 0.01:
        ttc = lead_one.dRel / car_state.vEgo
        self._dangerous_ttc_gmac.add_data(ttc)
        dynamic_threshold = self._get_dynamic_ttc_threshold()
        self._has_dangerous_ttc = (self._dangerous_ttc_gmac.get_moving_average() is not None and
                                self._dangerous_ttc_gmac.get_moving_average() <= dynamic_threshold)
    # keep prev values
    self._has_standstill_prev = self._has_standstill
    self._has_lead_filtered_prev = self._has_lead_filtered
    self._frame += 1

  def _radarless_mode(self):
    # when mpc fcw crash prob is high
    # use blended to slow down quickly
    if self._has_mpc_fcw:
      self._set_mode('blended')
      return

    # when blinker is on and speed is driving below V_ACC_MIN: blended
    # we dont want it to switch mode at higher speed, blended may trigger hard brake
    if self._has_blinkers and self._v_ego_kph < V_ACC_MIN:
      self._set_mode('blended')
      return

    # when at highway cruise and SNG: blended
    # ensuring blended mode is used because acc is bad at catching SNG lead car
    # especially those who accel very fast and then brake very hard.
    if self._sng_state == SNG_State.going and self._v_cruise_kph >= HIGHWAY_CRUISE_KPH:
      self._set_mode('blended')
      return

    # when standstill: blended
    # in case of lead car suddenly move away under traffic light, acc mode wont brake at traffic light.
    if self._has_standstill:
      self._set_mode('blended')
      return

    # when detecting slow down scenario: blended
    # e.g. traffic light, curve, stop sign etc.
    if self._has_slow_down:
      self._set_mode('blended')
      return

    # when detecting lead slow down: blended
    # use blended for higher braking capability
    if self._has_dangerous_ttc:
      self._set_mode('blended')
      return

    # car driving at speed lower than set speed: acc
    if self._has_slowness:
      self._set_mode('acc')
      return

    self._set_mode('acc')

  def _radar_mode(self):
    # when mpc fcw crash prob is high
    # use blended to slow down quickly
    if self._has_mpc_fcw:
      self._set_mode('blended')
      return

    # If there is a filtered lead, the vehicle is not in standstill, and the lead vehicle's yRel meets the condition,
    if self._has_lead_filtered and not self._has_standstill:
      self._set_mode('acc')
      return

    # when blinker is on and speed is driving below highway cruise speed: blended
    # we dont want it to switch mode at higher speed, blended may trigger hard brake
    if self._has_blinkers and self._v_ego_kph < HIGHWAY_CRUISE_KPH:
      self._set_mode('blended')
      return

    # when standstill: blended
    # in case of lead car suddenly move away under traffic light, acc mode wont brake at traffic light.
    if self._has_standstill:
      self._set_mode('blended')
      return

    # when detecting slow down scenario: blended
    # e.g. traffic light, curve, stop sign etc.
    if self._has_slow_down:
      self._set_mode('blended')
      return

    # car driving at speed lower than set speed: acc
    if self._has_slowness:
      self._set_mode('acc')
      return

    self._set_mode('acc')

  def get_mpc_mode(self, radar_unavailable, car_state, lead_one, md, controls_state):
    if self._is_enabled:
      self._update(car_state, lead_one, md, controls_state)
      if radar_unavailable:
        self._radarless_mode()
      else:
        self._radar_mode()

    self._mode_prev = self._mode
    return self._mode

  def set_enabled(self, enabled):
    self._is_enabled = enabled

  def is_enabled(self):
    return self._is_enabled

  def set_mpc_fcw_crash_cnt(self, crash_cnt):
    self._mpc_fcw_crash_cnt = crash_cnt

  def _set_mode(self, mode):
        current_time = time.monotonic()
        if not self._check_mode_switch_conditions(current_time):
            return
        if mode != self._mode:
            self._last_mode_switch = current_time
            self._mode = mode
            if mode == "blended":
                self._set_mode_timeout = SET_MODE_TIMEOUT
        if self._set_mode_timeout > 0:
            self._set_mode_timeout -= 1
