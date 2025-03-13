from cereal import log

class LateralParams:
    def __init__(self, personality=log.LongitudinalPersonality.standard):
        # 从容模式参数
        self.RELAXED = {
            'PATH_WEIGHT': 1.2,
            'HEADING_WEIGHT': 1.0,
            'STEER_RATE_WEIGHT': 1.5,
            'CURVE_FACTOR': 0.25,
            'MAX_ANGLE': 80,  # 最大转向角度(度)
            'MAX_CURVATURE': 45  # 最大曲率(度)
        }
        
        # 标准模式参数
        self.STANDARD = {
            'PATH_WEIGHT': 1.0,
            'HEADING_WEIGHT': 1.0,
            'STEER_RATE_WEIGHT': 1.0,
            'CURVE_FACTOR': 0.2,
            'MAX_ANGLE': 90,
            'MAX_CURVATURE': 50
        }
        
        # 激进模式参数
        self.AGGRESSIVE = {
            'PATH_WEIGHT': 0.8,
            'HEADING_WEIGHT': 1.2,
            'STEER_RATE_WEIGHT': 0.8,
            'CURVE_FACTOR': 0.15,
            'MAX_ANGLE': 90,
            'MAX_CURVATURE': 50
        }
        
        # 根据驾驶风格选择参数
        if personality == log.LongitudinalPersonality.relaxed:
            self.current = self.RELAXED
        elif personality == log.LongitudinalPersonality.aggressive:
            self.current = self.AGGRESSIVE
        else:
            self.current = self.STANDARD