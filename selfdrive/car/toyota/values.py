import re
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum, IntFlag#, StrEnum
from strenum import StrEnum
from typing import Dict, List, Set, Union
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import AngleRateLimit, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarFootnote, CarInfo, Column, CarParts, CarHarness
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS # 最小ACC(自适应巡航控制)速度 - 19英里/小时转换为米/秒
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS # 踏板过渡速度 - 10英里/小时转换为米/秒


class CarControllerParams:
  ACCEL_MAX = 1.5  # 最大加速度 - 1.5米/秒², 低于允许的2.0米/秒²用于调优
  ACCEL_MIN = -3.5   # 最小加速度(最大减速度) - -3.5米/秒²
  STEER_STEP = 1   # 转向步进值
  STEER_MAX = 1500 # 最大转向力矩
  STEER_ERROR_MAX = 350  # 最大转向误差 - 转向命令和实际转向力矩之间的最大差值

  # Lane Tracing Assist (LTA) control limits
  # Assuming a steering ratio of 13.7:
  # Limit to ~2.0 m/s^3 up (7.5 deg/s), ~3.5 m/s^3 down (13 deg/s) at 75 mph
  # Worst case, the low speed limits will allow ~4.0 m/s^3 up (15 deg/s) and ~4.9 m/s^3 down (18 deg/s) at 75 mph,
  # however the EPS has its own internal limits at all speeds which are less than that:
  # Observed internal torque rate limit on TSS 2.5 Camry and RAV4 is ~1500 units/sec up and down when using LTA
  # 车道追踪辅助（LTA）控制限制
  # 假设转向比为13.7:
  # 在75英里/小时的速度下，限制为向上约2.0米/秒^3（7.5度/秒），向下约3.5米/秒^3（13度/秒）
  # 在最坏的情况下，低速限制将允许在75英里/小时的速度下向上行驶约4.0米/秒^3（15度/秒），向下行驶约4.9米/秒^3（18度/秒），
  # 然而，EPS在所有低于该速度的速度下都有自己的内部限制：
  # 使用LTA时，TSS 2.5凯美瑞和RAV4上下观察到的内部扭矩率限制约为1500单位/秒
  # 向上角速度限制 - 在不同速度下的转向角速度限制
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.3, 0.15])
  # 向下角速度限制 - 在不同速度下的转向角速度限制
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.36, 0.26])

  def __init__(self, CP):
    if CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT:
      self.ACCEL_MAX = 2.0
    else:
      self.ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s^2 for tuning reasons
    self.ACCEL_MIN = -3.5  # m/s2
    if CP.lateralTuning.which() == 'torque':
      # 转向力矩上升速率 - 达到峰值力矩需要1.0秒
      self.STEER_DELTA_UP = 15       # 1.0s time to peak torque
      # 转向力矩下降速率 - 必须低于45，否则RAV4会报错(普锐斯最高可到50)
      self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
    else:
      self.STEER_DELTA_UP = 10       # 1.5s time to peak torque
      self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)


class ToyotaSafetyFlags(IntFlag):
  # first byte is for EPS scaling factor
  ALT_BRAKE = (1 << 8)
  STOCK_LONGITUDINAL = (2 << 8)
  LTA = (4 << 8)
  SECOC = (8 << 8)

class ToyotaFlags(IntFlag):
  HYBRID = 1
  SMART_DSU = 2
  DISABLE_RADAR = 4
  # Static flags
  TSS2 = 8
  NO_DSU = 16
  UNSUPPORTED_DSU = 32
  RADAR_ACC = 64
  # these cars use the Lane Tracing Assist (LTA) message for lateral control
  ANGLE_CONTROL = 128
  NO_STOP_TIMER = 256
  # these cars are speculated to allow stop and go when the DSU is unplugged
  SNG_WITHOUT_DSU = 512
  # these cars can utilize 2.0 m/s^2
  RAISED_ACCEL_LIMIT = 1024
  SECOC = 2048

class CAR(StrEnum):
  # Toyota
  ALPHARD_TSS2 = "TOYOTA ALPHARD 2020"
  AVALON = "TOYOTA AVALON 2016"
  AVALON_2019 = "TOYOTA AVALON 2019"
  AVALON_TSS2 = "TOYOTA AVALON 2022"  # TSS 2.5
  CAMRY = "TOYOTA CAMRY 2018"
  CAMRY_TSS2 = "TOYOTA CAMRY 2021"  # TSS 2.5
  CHR = "TOYOTA C-HR 2018"
  CHR_TSS2 = "TOYOTA C-HR 2021"
  COROLLA = "TOYOTA COROLLA 2017"
  # LSS2 Lexus UX Hybrid is same as a TSS2 Corolla Hybrid
  COROLLA_TSS2 = "TOYOTA COROLLA TSS2 2019"
  HIGHLANDER = "TOYOTA HIGHLANDER 2017"
  HIGHLANDER_TSS2 = "TOYOTA HIGHLANDER 2020"
  PRIUS = "TOYOTA PRIUS 2017"
  PRIUS_V = "TOYOTA PRIUS v 2017"
  PRIUS_TSS2 = "TOYOTA PRIUS TSS2 2021"
  RAV4 = "TOYOTA RAV4 2017"
  RAV4H = "TOYOTA RAV4 HYBRID 2017"
  RAV4_TSS2 = "TOYOTA RAV4 2019"
  RAV4_TSS2_2022 = "TOYOTA RAV4 2022"
  RAV4_TSS2_2023 = "TOYOTA RAV4 2023"
  RAV4_PRIME = "TOYOTA RAV4 PRIME 2021"
  MIRAI = "TOYOTA MIRAI 2021"  # TSS 2.5
  SIENNA = "TOYOTA SIENNA 2018"
  SIENNA_4TH_GEN = "TOYOTA SIENNA 2021"

  # Lexus
  LEXUS_CTH = "LEXUS CT HYBRID 2018"
  LEXUS_ES = "LEXUS ES 2018"
  LEXUS_ES_TSS2 = "LEXUS ES 2019"
  LEXUS_IS = "LEXUS IS 2018"
  LEXUS_IS_TSS2 = "LEXUS IS 2023"
  LEXUS_NX = "LEXUS NX 2018"
  LEXUS_NX_TSS2 = "LEXUS NX 2020"
  LEXUS_LC_TSS2 = "LEXUS LC 2024"
  LEXUS_RC = "LEXUS RC 2020"
  LEXUS_RX = "LEXUS RX 2016"
  LEXUS_RX_TSS2 = "LEXUS RX 2020"
  LEXUS_GS_F = "LEXUS GS F 2016"


class Footnote(Enum):
  CAMRY = CarFootnote(
    "openpilot operates above 28mph for Camry 4CYL L, 4CYL LE and 4CYL SE which don't have Full-Speed Range Dynamic Radar Cruise Control.",
    Column.FSR_LONGITUDINAL)


@dataclass
class ToyotaCarInfo(CarInfo):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.toyota_a]))


CAR_INFO: Dict[str, Union[ToyotaCarInfo, List[ToyotaCarInfo]]] = {
  # Toyota
  CAR.ALPHARD_TSS2: [
    ToyotaCarInfo("Toyota Alphard 2019-20"),
    ToyotaCarInfo("Toyota Alphard Hybrid 2021"),
  ],
  CAR.AVALON: [
    ToyotaCarInfo("Toyota Avalon 2016", "Toyota Safety Sense P"),
    ToyotaCarInfo("Toyota Avalon 2017-18"),
  ],
  CAR.AVALON_2019: [
    ToyotaCarInfo("Toyota Avalon 2019-21"),
    ToyotaCarInfo("Toyota Avalon Hybrid 2019-21"),
  ],
  CAR.AVALON_TSS2: [
    ToyotaCarInfo("Toyota Avalon 2022"),
    ToyotaCarInfo("Toyota Avalon Hybrid 2022"),
  ],
  CAR.CAMRY: [
    ToyotaCarInfo("Toyota Camry 2018-20", video_link="https://www.youtube.com/watch?v=fkcjviZY9CM", footnotes=[Footnote.CAMRY]),
    ToyotaCarInfo("Toyota Camry Hybrid 2018-20", video_link="https://www.youtube.com/watch?v=Q2DYY0AWKgk"),
  ],
  CAR.CAMRY_TSS2: [
    ToyotaCarInfo("Toyota Camry 2021-24", footnotes=[Footnote.CAMRY]),
    ToyotaCarInfo("Toyota Camry Hybrid 2021-24"),
  ],
  CAR.CHR: [
    ToyotaCarInfo("Toyota C-HR 2017-20"),
    ToyotaCarInfo("Toyota C-HR Hybrid 2017-20"),
  ],
  CAR.CHR_TSS2: [
    ToyotaCarInfo("Toyota C-HR 2021"),
    ToyotaCarInfo("Toyota C-HR Hybrid 2021-22"),
  ],
  CAR.COROLLA: ToyotaCarInfo("Toyota Corolla 2017-19"),
  CAR.COROLLA_TSS2: [
    ToyotaCarInfo("Toyota Corolla 2020-22", video_link="https://www.youtube.com/watch?v=_66pXk0CBYA"),
    ToyotaCarInfo("Toyota Corolla Cross (Non-US only) 2020-23", min_enable_speed=7.5),
    ToyotaCarInfo("Toyota Corolla Hatchback 2019-22", video_link="https://www.youtube.com/watch?v=_66pXk0CBYA"),
    # Hybrid platforms
    ToyotaCarInfo("Toyota Corolla Hybrid 2020-22"),
    ToyotaCarInfo("Toyota Corolla Hybrid (Non-US only) 2020-23", min_enable_speed=7.5),
    ToyotaCarInfo("Toyota Corolla Cross Hybrid (Non-US only) 2020-22", min_enable_speed=7.5),
    ToyotaCarInfo("Lexus UX Hybrid 2019-23"),
  ],
  CAR.HIGHLANDER: [
    ToyotaCarInfo("Toyota Highlander 2017-19", video_link="https://www.youtube.com/watch?v=0wS0wXSLzoo"),
    ToyotaCarInfo("Toyota Highlander Hybrid 2017-19"),
  ],
  CAR.HIGHLANDER_TSS2: [
    ToyotaCarInfo("Toyota Highlander 2020-23"),
    ToyotaCarInfo("Toyota Highlander Hybrid 2020-23"),
  ],
  CAR.PRIUS: [
    ToyotaCarInfo("Toyota Prius 2016", "Toyota Safety Sense P", video_link="https://www.youtube.com/watch?v=8zopPJI8XQ0"),
    ToyotaCarInfo("Toyota Prius 2017-20", video_link="https://www.youtube.com/watch?v=8zopPJI8XQ0"),
    ToyotaCarInfo("Toyota Prius Prime 2017-20", video_link="https://www.youtube.com/watch?v=8zopPJI8XQ0"),
  ],
  CAR.PRIUS_V: ToyotaCarInfo("Toyota Prius v 2017", "Toyota Safety Sense P", min_enable_speed=MIN_ACC_SPEED),
  CAR.PRIUS_TSS2: [
    ToyotaCarInfo("Toyota Prius 2021-22", video_link="https://www.youtube.com/watch?v=J58TvCpUd4U"),
    ToyotaCarInfo("Toyota Prius Prime 2021-22", video_link="https://www.youtube.com/watch?v=J58TvCpUd4U"),
  ],
  CAR.RAV4: [
    ToyotaCarInfo("Toyota RAV4 2016", "Toyota Safety Sense P"),
    ToyotaCarInfo("Toyota RAV4 2017-18")
  ],
  CAR.RAV4H: [
    ToyotaCarInfo("Toyota RAV4 Hybrid 2016", "Toyota Safety Sense P", video_link="https://youtu.be/LhT5VzJVfNI?t=26"),
    ToyotaCarInfo("Toyota RAV4 Hybrid 2017-18", video_link="https://youtu.be/LhT5VzJVfNI?t=26")
  ],
  CAR.RAV4_TSS2: [
    ToyotaCarInfo("Toyota RAV4 2019-21", video_link="https://www.youtube.com/watch?v=wJxjDd42gGA"),
    ToyotaCarInfo("Toyota RAV4 Hybrid 2019-21"),
  ],
  CAR.RAV4_TSS2_2022: [
    ToyotaCarInfo("Toyota RAV4 2022"),
    ToyotaCarInfo("Toyota RAV4 Hybrid 2022", video_link="https://youtu.be/U0nH9cnrFB0"),
  ],
  CAR.RAV4_TSS2_2023: [
    ToyotaCarInfo("Toyota RAV4 2023-24"),
    ToyotaCarInfo("Toyota RAV4 Hybrid 2023-25", video_link="https://youtu.be/4eIsEq4L4Ng"),
  ],
  CAR.RAV4_PRIME: ToyotaCarInfo("Toyota RAV4 Prime 2021-23"),
  CAR.MIRAI: ToyotaCarInfo("Toyota Mirai 2021"),
  CAR.SIENNA: ToyotaCarInfo("Toyota Sienna 2018-20", video_link="https://www.youtube.com/watch?v=q1UPOo4Sh68", min_enable_speed=MIN_ACC_SPEED),
  CAR.SIENNA_4TH_GEN: ToyotaCarInfo("Toyota Sienna 2021-23"),

  # Lexus
  CAR.LEXUS_CTH: ToyotaCarInfo("Lexus CT Hybrid 2017-18", "Lexus Safety System+"),
  CAR.LEXUS_ES: [
    ToyotaCarInfo("Lexus ES 2017-18"),
    ToyotaCarInfo("Lexus ES Hybrid 2017-18"),
  ],
  CAR.LEXUS_ES_TSS2: [
    ToyotaCarInfo("Lexus ES 2019-24"),
    ToyotaCarInfo("Lexus ES Hybrid 2019-24", video_link="https://youtu.be/BZ29osRVJeg?t=12"),
  ],
  CAR.LEXUS_IS: ToyotaCarInfo("Lexus IS 2017-19"),
  CAR.LEXUS_IS_TSS2: ToyotaCarInfo("Lexus IS 2022-23"),
  CAR.LEXUS_GS_F: ToyotaCarInfo("Lexus GS F 2016"),
  CAR.LEXUS_NX: [
    ToyotaCarInfo("Lexus NX 2018-19"),
    ToyotaCarInfo("Lexus NX Hybrid 2018-19"),
  ],
  CAR.LEXUS_NX_TSS2: [
    ToyotaCarInfo("Lexus NX 2020-21"),
    ToyotaCarInfo("Lexus NX Hybrid 2020-21"),
  ],
  CAR.LEXUS_LC_TSS2: ToyotaCarInfo("Lexus LC 2024"),
  CAR.LEXUS_RC: ToyotaCarInfo("Lexus RC 2018-20"),
  CAR.LEXUS_RX: [
    ToyotaCarInfo("Lexus RX 2016", "Lexus Safety System+"),
    ToyotaCarInfo("Lexus RX 2017-19"),
    # Hybrid platforms
    ToyotaCarInfo("Lexus RX Hybrid 2016", "Lexus Safety System+"),
    ToyotaCarInfo("Lexus RX Hybrid 2017-19"),
  ],
  CAR.LEXUS_RX_TSS2: [
    ToyotaCarInfo("Lexus RX 2020-22"),
    ToyotaCarInfo("Lexus RX Hybrid 2020-22"),
  ],
}

# (addr, cars, bus, 1/freq*100, vl)
STATIC_DSU_MSGS = [
  (0x128, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.AVALON), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x128, (CAR.HIGHLANDER, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES), 1,   3, b'\x03\x00\x20\x00\x00\x52'),
  (0x141, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON,
           CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON,
           CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.AVALON, CAR.PRIUS_V),
                                                                                               1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x161, (CAR.HIGHLANDER, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES), 1,  7, b'\x00\x1e\x00\xd4\x00\x00\x5b'),
  (0x283, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON,
           CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x2E6, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX), 0,   3, b'\xff\xf8\x00\x08\x7f\xe0\x00\x4e'),
  (0x2E7, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX), 0,   3, b'\xa8\x9c\x31\x9c\x00\x00\x00\x02'),
  (0x33E, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX), 0,  20, b'\x0f\xff\x26\x40\x00\x1f\x00'),
  (0x344, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON,
           CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, (CAR.PRIUS, CAR.LEXUS_NX, CAR.HIGHLANDER), 0,  20, b'\x00\x00\x00\x80\x03\x00\x08'),
  (0x365, (CAR.RAV4, CAR.RAV4H, CAR.COROLLA, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.LEXUS_RX,
           CAR.PRIUS_V), 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.HIGHLANDER), 0,  20, b'\x00\x00\x4d\x82\x40\x02\x00'),
  (0x366, (CAR.RAV4, CAR.COROLLA, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V),
          0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x470, (CAR.PRIUS, CAR.LEXUS_RX), 1, 100, b'\x00\x00\x02\x7a'),
  (0x470, (CAR.HIGHLANDER, CAR.RAV4H, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 1,  100, b'\x00\x00\x01\x79'),
  (0x4CB, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RX, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON,
           CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ES, CAR.PRIUS_V), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]


def get_platform_codes(fw_versions: List[bytes]) -> Dict[bytes, Set[bytes]]:
  """
  获取平台代码
  将固件版本信息解析为平台代码字典，用于在部件-平台-主版本组合内进行比较
  """
  # 返回子版本字典，用于在部件-平台-主版本组合内进行比较
  # Returns sub versions in a dict so comparisons can be made within part-platform-major_version combos
  codes = defaultdict(set)  # Optional[part]-platform-major_version: set of sub_version
  for fw in fw_versions:
    # UDS查询返回的固件版本可能包含多个数据块(不同ECU校准、不同数据?)
    # 并以一个字节作为前缀描述有多少数据块
    # 但KWP返回的固件需要查询每个子数据ID，没有长度前缀
    # FW versions returned from UDS queries can return multiple fields/chunks of data (different ECU calibrations, different data?)
    #  and are prefixed with a byte that describes how many chunks of data there are.
    # But FW returned from KWP requires querying of each sub-data id and does not have a length prefix.

    # 解析长度代码
    length_code = 1
    length_code_match = FW_LEN_CODE.search(fw)
    if length_code_match is not None:
      length_code = length_code_match.group()[0]
      fw = fw[1:]
    # 固件长度应该是16字节的倍数(每个块，即使没有长度代码)，长度不符则跳过解析
    # fw length should be multiple of 16 bytes (per chunk, even if no length code), skip parsing if unexpected length
    if length_code * FW_CHUNK_LEN != len(fw):
      continue
    # 分割数据块并去除填充字节
    chunks = [fw[FW_CHUNK_LEN * i:FW_CHUNK_LEN * i + FW_CHUNK_LEN].strip(b'\x00 ') for i in range(length_code)]
    # 目前只考虑第一个块，因为第二个通常是共享的
    # only first is considered for now since second is commonly shared (TODO: understand that)
    first_chunk = chunks[0]
    # 根据块长度使用不同的匹配模式
    if len(first_chunk) == 8:
      # 短格式：没有部件号，但某些短块在后续块中有
      # TODO: no part number, but some short chunks have it in subsequent chunks
      fw_match = SHORT_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((platform, major_version))].add(sub_version)
    # 中等格式
    elif len(first_chunk) == 10:
      fw_match = MEDIUM_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        part, platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((part, platform, major_version))].add(sub_version)
    # 长格式
    elif len(first_chunk) == 12:
      fw_match = LONG_FW_PATTERN.search(first_chunk)
      if fw_match is not None:
        part, platform, major_version, sub_version = fw_match.groups()
        codes[b'-'.join((part, platform, major_version))].add(sub_version)

  return dict(codes)


def match_fw_to_car_fuzzy(live_fw_versions, offline_fw_versions) -> Set[str]:
  """
  模糊匹配固件版本到车型
  比较在线和离线固件版本，找出匹配的车型
  """
  candidates = set()

  for candidate, fws in offline_fw_versions.items():
    # 跟踪通过所有检查的ECU(平台代码、子版本范围内)
    # Keep track of ECUs which pass all checks (platform codes, within sub-version range)
    valid_found_ecus = set()
    # 获取预期应该有平台代码的ECU地址集合
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # 只检查预期有平台代码的ECU
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & versions
      expected_platform_codes = get_platform_codes(expected_versions)

      # Found platform codes & versions
      found_platform_codes = get_platform_codes(live_fw_versions.get(addr, set()))
      # 检查部件号+平台代码+主版本号是否匹配
      # 平台代码和主版本号会随不同的物理部件、代际、API等变化
      # 子版本号用于小型召回更新，不需要检查
      # Check part number + platform code + major version matches for any found versions
      # Platform codes and major versions change for different physical parts, generation, API, etc.
      # Sub-versions are incremented for minor recalls, do not need to be checked.
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      valid_found_ecus.add(addr)
    # 如果所有在线ECU都通过了候选车型的所有检查，将其添加为匹配项
    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return {str(c) for c in (candidates - FUZZY_EXCLUDED_PLATFORMS)}

# 用于从固件版本解析平台特定标识符的正则表达式模式
# - 部件号：丰田部件号(通常需要忽略最后一个字符才能找到匹配)
#   每个ECU地址只有一个部件号
# - 平台：每个openpilot平台通常有多个代码，但这是变化最小的
#   通常在ECU和年款之间共享，表示特定平台的某些特征
#   描述更多的代际变化(TSS-P vs TSS2)或制造地区
# - 主版本号：固件版本中第二不变的部分。用于区分按型号年份/API划分的车型
#   如RAV4 2022/2023和Avalon。用于区分API略有变化但不是代际变化的车型
# - 子版本号：专属于主版本号，但与其他车型共享。仅用于进一步过滤
#   在TSB固件更新中会增加，描述其他细微差异
# Regex patterns for parsing more general platform-specific identifiers from FW versions.
# - Part number: Toyota part number (usually last character needs to be ignored to find a match).
#    Each ECU address has just one part number.
# - Platform: usually multiple codes per an openpilot platform, however this is the least variable and
#    is usually shared across ECUs and model years signifying this describes something about the specific platform.
#    This describes more generational changes (TSS-P vs TSS2), or manufacture region.
# - Major version: second least variable part of the FW version. Seen splitting cars by model year/API such as
#    RAV4 2022/2023 and Avalon. Used to differentiate cars where API has changed slightly, but is not a generational change.
#    It is important to note that these aren't always consecutive, for example:
#    Avalon 2016-18's fwdCamera has these major versions: 01, 03 while 2019 has: 02
# - Sub version: exclusive to major version, but shared with other cars. Should only be used for further filtering.
#    Seen bumped in TSB FW updates, and describes other minor differences.
SHORT_FW_PATTERN = re.compile(b'[A-Z0-9](?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{2})(?P<sub_version>[A-Z0-9]{3})')
MEDIUM_FW_PATTERN = re.compile(b'(?P<part>[A-Z0-9]{5})(?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{1})(?P<sub_version>[A-Z0-9]{2})')
LONG_FW_PATTERN = re.compile(b'(?P<part>[A-Z0-9]{5})(?P<platform>[A-Z0-9]{2})(?P<major_version>[A-Z0-9]{2})(?P<sub_version>[A-Z0-9]{3})')
FW_LEN_CODE = re.compile(b'^[\x01-\x03]')  # highest seen is 3 chunks, 16 bytes each
FW_CHUNK_LEN = 16

# 列出在openpilot平台中最具特色的ECU单元
# - fwdCamera(前置摄像头): 描述与ADAS相关的实际功能。例如，在Avalon车型上，它描述了：
#    TSS-P何时成为标配、车辆是否支持停走功能、是否为TSS2系统。
#    在RAV4上，它描述了雷达执行ACC的变化，以及使用LTA进行车道保持。
#    注意：平台代码和主版本号并不直接描述功能，只能通过与数据库中其他已知固件版本匹配来推断功能。
# - fwdRadar(前置雷达): 用于对前置摄像头进行合理性检查，通常共享相同的平台代码。
#    例如，2022款RAV4的新雷达架构在两者的平台代码中都有体现。
# - abs(防抱死制动系统): 用于区分大多数车型的混动/燃油版本（TSS2版本的卡罗拉是个例外，由于混合动力平台组合原因未使用）
# - eps(电动助力转向): 描述EPS的横向控制API变化，例如使用LTA进行车道保持和拒绝LKA消息
PLATFORM_CODE_ECUS = (Ecu.fwdCamera, Ecu.fwdRadar, Ecu.eps)

# 这些平台的所有ECU至少有一个与其他平台共享的平台代码
# rick - 快速修复错误
from typing import Set
FUZZY_EXCLUDED_PLATFORMS: Set[CAR] = set()  # 模糊匹配时需要排除的平台集合

# 一些使用KWP2000协议的ECU的固件版本存储在非标准数据标识符中
# 丰田诊断软件首先获取支持的数据ID列表，然后逐个查询这些ID
# 例如：发送请求: 0x1a8800, 接收响应: 0x1a8800010203,
# 然后依次查询: 0x1a8801, 0x1a8802, 0x1a8803
TOYOTA_VERSION_REQUEST_KWP = b'\x1a\x88\x01'   # KWP协议版本请求命令
TOYOTA_VERSION_RESPONSE_KWP = b'\x5a\x88\x01'  # KWP协议版本响应命令

FW_QUERY_CONFIG = FwQueryConfig(
  # TODO: look at data to whitelist new ECUs effectively
  requests=[
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, TOYOTA_VERSION_REQUEST_KWP],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, TOYOTA_VERSION_RESPONSE_KWP],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.fwdRadar, Ecu.dsu, Ecu.abs, Ecu.eps, Ecu.epb, Ecu.telematics,
                      Ecu.srs, Ecu.combinationMeter, Ecu.transmission, Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, StdQueries.OBD_VERSION_REQUEST],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, StdQueries.OBD_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.epb, Ecu.telematics, Ecu.hybrid, Ecu.srs, Ecu.combinationMeter, Ecu.transmission,
                      Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.DEFAULT_DIAGNOSTIC_REQUEST, StdQueries.EXTENDED_DIAGNOSTIC_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE, StdQueries.EXTENDED_DIAGNOSTIC_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.abs, Ecu.eps, Ecu.epb, Ecu.telematics,
                      Ecu.hybrid, Ecu.srs, Ecu.combinationMeter, Ecu.transmission, Ecu.gateway, Ecu.hvac],
      bus=0,
    ),
  ],
  non_essential_ecus={
    # FIXME: On some models, abs can sometimes be missing
    Ecu.abs: [CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.SIENNA, CAR.LEXUS_IS, CAR.ALPHARD_TSS2],
    # On some models, the engine can show on two different addresses
    Ecu.engine: [CAR.HIGHLANDER, CAR.CAMRY, CAR.COROLLA_TSS2, CAR.CHR, CAR.CHR_TSS2, CAR.LEXUS_IS,
                 CAR.LEXUS_RC, CAR.LEXUS_NX, CAR.LEXUS_NX_TSS2, CAR.LEXUS_RX, CAR.LEXUS_RX_TSS2],
  },
  extra_ecus=[
    # 以下是后期丰田车型上未在此查询的所有已知ECU:
    # 支持UDS协议的ECU:
    # - 混动电池 (0x713, 0x747)
    # - 电机发电机 (0x716, 0x724)
    # - 第二ABS"制动/EPB" (0x730)
    # 支持KWP协议(0x1a8801)的ECU:
    # - 转向角传感器 (0x7b3)
    # - EPS/EMPS电动助力转向 (0x7a0, 0x7a1)
    # 支持KWP协议(0x1a8881)的ECU:
    # - 车身控制模块 ((0x750, 0x40))

    # 混动控制电脑可能在0x7e2(KWP)或0x7d2(UDS)地址，取决于平台
    (Ecu.hybrid, 0x7e2, None),  # 混动系统控制总成和电脑

    # TODO: 如果这些重复的ECU总是同时存在，删除其中一个
    (Ecu.srs, 0x780, None),     # SRS安全气囊系统1
    (Ecu.srs, 0x784, None),     # SRS安全气囊系统2

    # 可能仅存在于EPB不是标配的车型(如凯美瑞、亚洲龙及其混动版)
    # 在某些车型上，EPB由ABS模块控制
    (Ecu.epb, 0x750, 0x2c),     # 电子驻车制动系统

    # 此ECU并非所有车型都能访问
    (Ecu.gateway, 0x750, 0x5f),  # 网关模块

    # 在某些车型上，此ECU仅响应b'\x1a\x88\x81'命令，这反映在b'\x1a\x88\x00'查询中
    (Ecu.telematics, 0x750, 0xc7),  # 远程信息处理系统

    # 在某些平台上(如TSS-P的RAV4)变速箱与发动机控制合并
    (Ecu.transmission, 0x701, None),  # 变速箱控制单元1
    # 部分平台在此地址有测试响应，添加到日志
    (Ecu.transmission, 0x7e1, None),  # 变速箱控制单元2

    # 在某些车型上，此ECU仅响应b'\x1a\x88\x80'命令
    (Ecu.combinationMeter, 0x7c0, None),  # 组合仪表
    (Ecu.hvac, 0x7c4, None),  # 空调控制系统
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)


STEER_THRESHOLD = 100

DBC = {
  CAR.RAV4H: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.RAV4: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
  CAR.PRIUS: dbc_dict('toyota_nodsu_pt_generated', 'toyota_adas'),
  CAR.PRIUS_V: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
  CAR.COROLLA: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
  CAR.LEXUS_RC: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.LEXUS_RX: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.LEXUS_RX_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.CHR: dbc_dict('toyota_nodsu_pt_generated', 'toyota_adas'),
  CAR.CHR_TSS2: dbc_dict('toyota_nodsu_pt_generated', None),
  CAR.CAMRY: dbc_dict('toyota_nodsu_pt_generated', 'toyota_adas'),
  CAR.CAMRY_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.HIGHLANDER: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.HIGHLANDER_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.AVALON: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.AVALON_2019: dbc_dict('toyota_nodsu_pt_generated', 'toyota_adas'),
  CAR.AVALON_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.RAV4_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.RAV4_TSS2_2022: dbc_dict('toyota_nodsu_pt_generated', None),
  CAR.RAV4_TSS2_2023: dbc_dict('toyota_nodsu_pt_generated', None),
  CAR.COROLLA_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.LEXUS_ES: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
  CAR.LEXUS_ES_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.SIENNA: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.LEXUS_IS: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.LEXUS_IS_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.LEXUS_CTH: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
  CAR.LEXUS_NX: dbc_dict('toyota_tnga_k_pt_generated', 'toyota_adas'),
  CAR.LEXUS_NX_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.PRIUS_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.MIRAI: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.ALPHARD_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.RAV4_PRIME: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.SIENNA_4TH_GEN: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.LEXUS_LC_TSS2: dbc_dict('toyota_nodsu_pt_generated', 'toyota_tss2_adas'),
  CAR.LEXUS_GS_F: dbc_dict('toyota_new_mc_pt_generated', 'toyota_adas'),
}

# 这些车型具有非标准的EPS(电动助力转向)力矩比例因子。其他车型都是73
EPS_SCALE = defaultdict(lambda: 73, {CAR.PRIUS: 66, CAR.COROLLA: 88, CAR.LEXUS_IS: 77, CAR.LEXUS_RC: 77, CAR.LEXUS_CTH: 100, CAR.PRIUS_V: 100})

# 配备丰田/雷克萨斯Safety Sense 2.0和2.5系统的车型
TSS2_CAR = {CAR.RAV4_TSS2, CAR.RAV4_TSS2_2022, CAR.RAV4_TSS2_2023, CAR.COROLLA_TSS2, CAR.LEXUS_ES_TSS2,
            CAR.LEXUS_RX_TSS2, CAR.HIGHLANDER_TSS2, CAR.PRIUS_TSS2, CAR.CAMRY_TSS2, CAR.LEXUS_IS_TSS2,
            CAR.MIRAI, CAR.LEXUS_NX_TSS2, CAR.ALPHARD_TSS2, CAR.AVALON_TSS2, CAR.CHR_TSS2,
            CAR.RAV4_PRIME, CAR.SIENNA_4TH_GEN, CAR.LEXUS_LC_TSS2}

# 不配备DSU(驾驶辅助单元)的车型，包括所有TSS2车型和部分其他车型
NO_DSU_CAR = TSS2_CAR | {CAR.CHR, CAR.CAMRY}

# 这些车型的DSU使用AEB(自动紧急制动)消息来进行纵向控制，目前不支持
UNSUPPORTED_DSU_CAR = {CAR.LEXUS_IS, CAR.LEXUS_RC, CAR.LEXUS_GS_F}

# 这些车型使用雷达发送ACC(自适应巡航控制)消息，而不是通过摄像头
RADAR_ACC_CAR = {CAR.RAV4_TSS2_2022, CAR.RAV4_TSS2_2023, CAR.CHR_TSS2}

# 这些车型使用LTA(车道追踪辅助)消息进行横向控制
ANGLE_CONTROL_CAR = {CAR.RAV4_TSS2_2023}

# 这些车型在停车后重新启动时不需要按下恢复按钮
NO_STOP_TIMER_CAR = TSS2_CAR | {CAR.PRIUS_V, CAR.RAV4H, CAR.HIGHLANDER, CAR.SIENNA}
