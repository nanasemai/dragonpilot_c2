import time
import json
import traceback
import sys
import os
from pathlib import Path
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

class PandaDataLogger:
    def __init__(self):
        self.params = Params()
        self.log_dir = Path("/data/media/0/c2_logs/panda_logs")
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # 状态追踪
        self.last_stats = {}
        self.last_error_time = 0
        self.error_cooldown = 60.0
        self.last_lkas_mono = 0 # 用于检测帧间隔

        self.thresholds = {
            'interrupt_load': 0.85,
            'voltage_low': 10500,  # 10.5V
            'packet_loss_max': 0.05,
            'frame_interval_max': 0.050 # 50ms 阈值
        }

    def _get_panda_snapshot(self, panda_states):
        """提取关键硬件指标"""
        snapshot = []
        for i, p in enumerate(panda_states):
            p_data = {
                "idx": i,
                "voltage": p.voltage,
                "load": p.interruptLoad,
                "tx_blocked": p.safetyTxBlocked,
                "heartbeat": not p.heartbeatLost,
                "buses": []
            }
            # C2 资源有限，只遍历前两个常用的总线以节省微量 CPU
            for j in range(2):
                can = getattr(p, f'canState{j}', None)
                if can:
                    p_data["buses"].append({
                        "bus": j,
                        "off": can.busOff,
                        "tx": can.totalTxCnt,
                        "lost": can.totalTxLostCnt
                    })
            snapshot.append(p_data)
        return snapshot

    def _detect_issues(self, snapshot, sm):
        """综合检测硬件和时序异常"""
        issues = []
        
        # 1. 硬件异常检测
        for p in snapshot:
            if 0 < p['voltage'] < self.thresholds['voltage_low']:
                issues.append(f"电压低({p['voltage']}mV)")
            if p['load'] > self.thresholds['interrupt_load']:
                issues.append(f"Panda负载极高({p['load']:.2f})")
            if not p['heartbeat']:
                issues.append("Panda心跳丢失")
            
            for b in p['buses']:
                if b['off']:
                    issues.append(f"Bus{b['bus']}总线关闭")
                
                # 增量丢包计算
                key = (p['idx'], b['bus'])
                if key in self.last_stats:
                    p_tx, p_lost = self.last_stats[key]
                    d_tx = b['tx'] - p_tx
                    d_lost = b['lost'] - p_lost
                    # 确保d_tx为正且有足够样本量
                    if d_tx > 0 and d_tx > 50:
                        rate = d_lost / d_tx
                        if rate > self.thresholds['packet_loss_max']:
                            issues.append(f"Bus{b['bus']}丢包严重({rate:.1%})")
                self.last_stats[key] = (b['tx'], b['lost'])

        # 2. LKAS 时序异常检测 (仅在车速 > 1.0 且 OP 激活时)
        # 这样可以精准定位导致转向闪退的系统卡顿
        try:
            cs = sm['carState']
            ctrl = sm['controlsState']
            if cs.vEgo > 1.0 and getattr(ctrl, 'active', False):
                # 确保logMonoTime可用
                if 'carState' in sm.logMonoTime:
                    curr_mono = sm.logMonoTime['carState']
                    # 确保curr_mono有效且与上次时间差合理
                    if self.last_lkas_mono > 0 and curr_mono > self.last_lkas_mono:
                        interval = (curr_mono - self.last_lkas_mono) / 1e9
                        # 限制异常间隔的最大范围，避免系统时间跳变导致的误报
                        if self.thresholds['frame_interval_max'] < interval < 5.0:
                            issues.append(f"LKAS帧间隔异常({interval:.3f}s)")
                    self.last_lkas_mono = curr_mono
        except Exception as e:
            cloudlog.error(f"LKAS帧间隔检测失败: {str(e)}")

        return issues

    def save_error_event(self, issues, snapshot):
        curr_time = time.time()
        if curr_time - self.last_error_time < self.error_cooldown:
            return

        payload = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "issues": issues,
            "snapshot": snapshot
        }
        
        file_path = self.log_dir / f"error_{int(curr_time)}.json"
        try:
            with open(file_path, 'w') as f:
                json.dump(payload, f, indent=2)
            self.last_error_time = curr_time
            cloudlog.error(f"Panda/LKAS异常已记录: {issues}")
        except:
            pass

    def cleanup_logs(self):
        try:
            files = sorted(list(self.log_dir.glob("*.json")), key=lambda x: x.stat().st_mtime)
            if len(files) > 50: # C2 存储紧张，保留50个报错文件足够
                for f in files[:-50]: f.unlink()
        except:
            pass

    def log_loop(self):
        os.nice(15)
        # 2Hz 频率对硬件监控来说是黄金平衡点
        rk = Ratekeeper(2)
        sm = messaging.SubMaster(['pandaStates', 'carState', 'controlsState'])

        while True:
            try:
                sm.update(0)
                
                if sm.updated['pandaStates'] and len(sm['pandaStates']) > 0:
                    snapshot = self._get_panda_snapshot(sm['pandaStates'])
                    issues = self._detect_issues(snapshot, sm)

                    if issues:
                        self.save_error_event(issues, snapshot)

                # 每半小时检查一次日志清理
                if rk.frame % 3600 == 0:
                    self.cleanup_logs()

                rk.keep_time()
            except Exception:
                cloudlog.error(f"Panda Logger Loop Error: {traceback.format_exc()}")
                time.sleep(2)

def main():
    time.sleep(15) # 避开启动峰值
    logger = PandaDataLogger()
    try:
        logger.log_loop()
    except Exception:
        error_msg = traceback.format_exc()
        cloudlog.error(f"PandaDataLogger CRASHED:\n{error_msg}")
        sys.exit(1)

if __name__ == "__main__":
    main()