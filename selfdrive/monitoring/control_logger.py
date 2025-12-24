import time
import json
import sys
import os
import traceback
from pathlib import Path

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

class ControlDataLogger:
    def __init__(self):
        params = Params()
        self.log_dir = Path("/data/media/0/c2_logs/control_logs")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # 只在初始化时读取一次参数
        try:
            self.nnff_enabled = params.get_bool("dp_use_nnff") or params.get_bool("dp_use_nnff_lite")
        except:
            self.nnff_enabled = False
            
        self.last_error_time = 0
        self.error_cooldown = 30.0

        self.thresholds = {
            'steering_error_max': 5.0,
            'accel_error_max': 0.6,
        }

    def _get_snapshot(self, cs, ctrl, actuators):
        """提取关键字段快照"""
        try:
            # 容错处理：确保 actuators 存在
            steer_out = getattr(actuators, 'steer', 0) if actuators else 0
            
            return {
                'vEgo': cs.vEgo,
                'steerAngle': cs.steeringAngleDeg,
                'desiredAngle': getattr(ctrl, 'desiredSteeringAngleDeg', 0),
                'steerOutput': steer_out,
                'accel': cs.aEgo,
                'desiredAccel': getattr(ctrl, 'aTarget', 0),
                'latActive': getattr(ctrl, 'latActive', False),
                'longActive': getattr(ctrl, 'longActive', False),
                'nnff_enabled': self.nnff_enabled, # 使用初始化的内存变量
            }
        except:
            return None

    def _detect_issues(self, snapshot):
        if not snapshot: return []
        issues = []
        
        # 1. 转向误差检测
        angle_err = abs(snapshot['steerAngle'] - snapshot['desiredAngle'])
        if snapshot['latActive'] and angle_err > self.thresholds['steering_error_max']:
            issues.append(f"转向误差过大({angle_err:.1f}°) ")

        # 2. NNFF 检测 - 改进条件
        if snapshot['nnff_enabled'] and snapshot['latActive'] and snapshot['vEgo'] > 1.0:
            # 如果误差大但控制输出却很小，说明 NNFF 补偿可能失效或不匹配
            # 增加车速条件，避免低速时误报
            if angle_err > 3.5 and abs(snapshot['steerOutput']) < 0.1:
                issues.append("NNFF表现异常(误差大但输出小)")

        # 3. 纵向误差
        accel_err = abs(snapshot['accel'] - snapshot['desiredAccel'])
        if snapshot['longActive'] and accel_err > self.thresholds['accel_error_max']:
            issues.append(f"加速度偏差大({accel_err:.2f})")

        return issues

    def save_error_event(self, issues, snapshot):
        curr_time = time.time()
        if curr_time - self.last_error_time < self.error_cooldown:
            return

        payload = {
            'time': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            'issues': issues,
            'data': snapshot
        }
        
        file_path = self.log_dir / f"error_{int(curr_time)}.json"
        try:
            with open(file_path, 'w') as f:
                json.dump(payload, f, indent=2)
            self.last_error_time = curr_time
            cloudlog.warning(f"检测到控制异常并记录: {file_path}")
            
            # 自动清理：仅保留最近 30 个报错文件
            files = sorted(list(self.log_dir.glob("error_*.json")), key=lambda x: x.stat().st_mtime)
            if len(files) > 30:
                for f in files[:-30]: f.unlink()
        except:
            pass

    def log_loop(self):
        # 降低优先级
        os.nice(15)
        # 5Hz 频率
        rk = Ratekeeper(5)
        sm = messaging.SubMaster(['carControl', 'carState', 'controlsState'])

        cloudlog.info(f"报错监控启动 (NNFF开关状态: {self.nnff_enabled})")

        while True:
            try:
                sm.update(0)
                
                if sm.updated['carState'] and sm.updated['controlsState']:
                    cs = sm['carState']
                    ctrl = sm['controlsState']

                    # 只有在辅助驾驶激活或行驶时才分析
                    if ctrl.active or cs.vEgo > 1.0:
                        # 即使carControl没有更新，也使用最新的actuators信息
                        actuators = sm['carControl'].actuators if hasattr(sm['carControl'], 'actuators') else None
                        snapshot = self._get_snapshot(cs, ctrl, actuators)
                        
                        if snapshot:
                            issues = self._detect_issues(snapshot)
                            if issues:
                                self.save_error_event(issues, snapshot)

                rk.keep_time()
            except Exception:
                cloudlog.error(f"Loop error: {traceback.format_exc()}")
                time.sleep(1)

def main():
    # 延迟 15 秒避开开机 CPU 峰值
    time.sleep(15)
    
    logger = ControlDataLogger()
    try:
        logger.log_loop()
    except Exception:
        error_msg = traceback.format_exc()
        cloudlog.error(f"control_monitoring CRASHED:\n{error_msg}")
        sys.exit(1)

if __name__ == "__main__":
    main()