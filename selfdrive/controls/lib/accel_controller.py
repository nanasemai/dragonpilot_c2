from openpilot.common.numpy_fast import interp

# 驾驶模式定义
DP_ACCEL_STOCK = 0   # 原厂模式：使用车辆原厂的加减速控制逻辑
DP_ACCEL_ECO = 1     # 经济模式：优先节能，加减速更平顺，适合日常通勤
DP_ACCEL_NORMAL = 2  # 标准模式：平衡舒适性和动力性，适合一般驾驶场景
DP_ACCEL_SPORT = 3   # 运动模式：提供更快的加速响应，适合运动驾驶

# accel profile by @arne182 modified by cgw
# 减速度控制参数配置（适配1.2T发动机特性）
# 数值为负数，表示最大允许减速度(m/s²)
_DP_CRUISE_MIN_V =       [-0.90,  -1.00,  -0.90,  -0.90,  -0.85,  -0.85,  -0.85]  # 标准模式减速度
_DP_CRUISE_MIN_V_ECO =   [-0.80,  -0.90,  -0.80,  -0.80,  -0.80,  -0.80,  -0.80]  # 经济模式减速度
_DP_CRUISE_MIN_V_SPORT = [-1.00,  -1.10,  -1.00,  -1.00,  -0.95,  -0.95,  -0.95]  # 运动模式减速度
# 减速度对应的速度断点 [m/s]        [km/h]
_DP_CRUISE_MIN_BP =      [0.,     # 0km/h   静止
                         0.3,    # 1km/h   低速起步
                         0.6,    # 2km/h   缓慢行驶
                         8.33,   # 30km/h  市区低速
                         16.,    # 58km/h  市区高速
                         30.,    # 108km/h 高速巡航
                         40.]    # 144km/h 高速最高

# 加速度控制参数配置（适配1.2T发动机特性）
# 数值为正数，表示最大允许加速度(m/s²)
_DP_CRUISE_MAX_V =       [2.0,  2.0,  1.8,  1.50,  0.95,  0.75,  0.55,  0.38,  0.30,  0.10]   # 标准模式加速度
_DP_CRUISE_MAX_V_ECO =   [1.8,  1.8,  1.6,  1.30,  0.85,  0.65,  0.45,  0.32,  0.25,  0.08]   # 经济模式加速度
_DP_CRUISE_MAX_V_SPORT = [2.3,  2.3,  2.0,  1.70,  1.10,  0.80,  0.65,  0.50,  0.35,  0.15]   # 运动模式加速度
# 加速度对应的速度断点 [m/s]        [km/h]
_DP_CRUISE_MAX_BP =      [0.,     # 0km/h   静止
                         1.,     # 3.6km/h 低速起步
                         6.,     # 22km/h  加速过渡
                         8.,     # 29km/h  市区低速
                         11.,    # 40km/h  市区中速
                         15.,    # 54km/h  市区高速
                         20.,    # 72km/h  郊区速度
                         25.,    # 90km/h  高速入口
                         30.,    # 108km/h 高速巡航
                         55.]    # 198km/h 高速最高

class AccelController:

  def __init__(self):
    # self._params = Params()
    self._profile = DP_ACCEL_STOCK

  def set_profile(self, profile):
    try:
      self._profile = int(profile) if int(profile) in [DP_ACCEL_STOCK, DP_ACCEL_ECO, DP_ACCEL_NORMAL, DP_ACCEL_SPORT] else DP_ACCEL_STOCK
    except:
      self._profile = DP_ACCEL_STOCK

  def _dp_calc_cruise_accel_limits(self, v_ego):
    if self._profile == DP_ACCEL_ECO:
      min_v = _DP_CRUISE_MIN_V_ECO
      max_v = _DP_CRUISE_MAX_V_ECO
    elif self._profile == DP_ACCEL_SPORT:
      min_v = _DP_CRUISE_MIN_V_SPORT
      max_v = _DP_CRUISE_MAX_V_SPORT
    else:
      min_v = _DP_CRUISE_MIN_V
      max_v = _DP_CRUISE_MAX_V

    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, min_v)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, max_v)
    return [a_cruise_min, a_cruise_max]

  def get_accel_limits(self, v_ego, accel_limits):
    if self._profile == DP_ACCEL_STOCK:
      return accel_limits
    
    custom_min, custom_max = self._dp_calc_cruise_accel_limits(v_ego)
    # 确保不会比原厂规定的安全制动限值更弱
    safe_min = min(custom_min, accel_limits[0])
    # 将加速度限制在舒适区间内，但不超过原厂最大值
    safe_max = min(custom_max, accel_limits[1])
    return [safe_min, safe_max]

  def is_enabled(self):
    return self._profile != DP_ACCEL_STOCK
