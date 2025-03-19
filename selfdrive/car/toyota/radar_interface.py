#!/usr/bin/env python3
"""
丰田车辆雷达接口模块
该模块负责处理来自车辆雷达的原始数据，并将其转换为标准化的雷达数据格式
主要功能：
- 解析雷达CAN消息
- 跟踪和管理雷达检测到的目标
- 提供标准化的雷达数据输出
"""
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.toyota.values import DBC, TSS2_CAR
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(car_fingerprint):
  """
  创建雷达CAN解析器
  参数:
    car_fingerprint: 车型指纹，用于确定雷达消息ID范围
  返回:
    配置好的CAN解析器实例
  """
  # TSS2车型使用不同的雷达消息ID范围
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))  # TSS2雷达A消息范围
    RADAR_B_MSGS = list(range(0x190, 0x1a0))  # TSS2雷达B消息范围
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))  # 传统雷达A消息范围
    RADAR_B_MSGS = list(range(0x220, 0x230))  # 传统雷达B消息范围

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  # 配置所有雷达消息的采样频率为20Hz
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n)))

  return CANParser(DBC[car_fingerprint]['radar'], messages, 1)

class RadarInterface(RadarInterfaceBase):
  """
  丰田雷达接口类
  处理雷达数据并提供标准化输出
  """
  def __init__(self, CP):
    """
    初始化雷达接口
    参数:
      CP: 车辆参数对象
    """
    super().__init__(CP)
    self.track_id = 0  # 雷达目标跟踪ID
    # 根据车型确定雷达消息ID范围
    if CP.carFingerprint in TSS2_CAR:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    # 初始化每个雷达点的有效计数器
    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    # 创建雷达CAN解析器（如果雷达可用）
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]  # 触发消息ID
    self.updated_messages = set()  # 已更新消息集合

  def update(self, can_strings):
    """
    更新雷达数据
    参数:
      can_strings: CAN总线数据字符串
    返回:
      处理后的雷达数据或None
    """
    if self.rcp is None:
      return super().update(None)

    # 更新CAN解析器并获取更新的消息ID
    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    # 等待触发消息
    if self.trigger_msg not in self.updated_messages:
      return None

    # 处理更新的消息并清空消息集合
    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    """
    处理雷达数据更新
    参数:
      updated_messages: 需要处理的消息ID集合
    返回:
      处理后的雷达数据
    """
    # 创建新的雷达数据消息
    ret = car.RadarData.new_message()
    errors = []
    
    # 简化错误检查逻辑但保持详细的错误信息
    if self.rcp is None:
      errors.append("radarUnavailable")
    elif not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    # 处理每个更新的消息
    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]
        # 数据有效性检查：距离、横向位置和相对速度
        if not (0 <= cpt['LONG_DIST'] < 255 and -50 <= cpt['LAT_DIST'] <= 50 and abs(cpt['REL_SPEED']) < 100):
          continue
        # 优化计数器更新逻辑
        if cpt['NEW_TRACK'] or cpt['LONG_DIST'] >= 255:
          self.valid_cnt[ii] = 0
        elif cpt['VALID']:
          self.valid_cnt[ii] = min(self.valid_cnt[ii] + 1, 10)
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)
        score = self.rcp.vl[ii+16]['SCORE']
        # 判断雷达点是否有效：测量有效或评分大于50且距离有效且计数器大于0
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          # 创建新的跟踪点或更新现有点
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          
          # 更新雷达点数据
          self.pts[ii].dRel = cpt['LONG_DIST']  # 纵向距离（相对于车辆前部）
          self.pts[ii].yRel = -cpt['LAT_DIST']  # 横向距离（车辆坐标系中，左为正）
          self.pts[ii].vRel = cpt['REL_SPEED']  # 相对速度
          self.pts[ii].aRel = float('nan')      # 相对加速度（暂不可用）
          self.pts[ii].yvRel = float('nan')     # 横向相对速度（暂不可用）
          self.pts[ii].measured = bool(cpt['VALID'])  # 是否为有效测量
        else:
          # 删除无效的雷达点
          if ii in self.pts:
            del self.pts[ii]

    # 返回所有有效的雷达点
    ret.points = list(self.pts.values())
    return ret
