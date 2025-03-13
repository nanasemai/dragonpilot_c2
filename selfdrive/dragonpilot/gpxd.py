import cereal.messaging as messaging
import datetime
import os
from pathlib import Path
from openpilot.common.realtime import set_core_affinity, set_realtime_priority
from openpilot.common.swaglog import cloudlog, add_file_handler

# 配置参数
GPX_LOG_PATH = '/data/media/0/gpx_logs/'
#GPX_ERRORLOGS_PATH = '/data/media/0/c2_logs/gpx_info_logs/'
LOG_HERTZ = 5                    # 采样频率 (Hz)
LOG_LENGTH = 5                   # 记录时长 (分钟)
LOST_SIGNAL_COUNT_LENGTH = 10    # 信号丢失判定时长 (秒)
MIN_SPEED_THRESHOLD = 0.1        # 最小速度阈值 (m/s)

# 计算常量
LOST_SIGNAL_COUNT_MAX = LOST_SIGNAL_COUNT_LENGTH * LOG_HERTZ
LOGS_PER_FILE = LOG_LENGTH * 60 * LOG_HERTZ

class GpxD:
  def __init__(self):
    self.log_count = 0
    self.logs = []
    self.lost_signal_count = 0
    self.started_time = datetime.datetime.utcnow().isoformat()
    self.pause = True

    # 确保日志目录存在
    #Path(GPX_ERRORLOGS_PATH).mkdir(parents=True, exist_ok=True)

    # 修改日志初始化方式，使用自定义日志目录
    cloudlog.bind_global(module='gpxd')
    self.session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cloudlog.bind(session_id=self.session_id)

  def log(self, sm):
    try:
      gps = sm['gpsLocationExternal']
      # 更新暂停状态时使用结构化日志
      if gps.speed >= MIN_SPEED_THRESHOLD:
        if self.pause:
          #cloudlog.info("记录开始", state="moving", speed=gps.speed, log_dir=GPX_ERRORLOGS_PATH)
          self.pause = False
      # 检查GPS信号有效性
      if not gps.flags % 2 or self.pause:
        if self.log_count > 0:
          self.lost_signal_count += 1
          if self.lost_signal_count % LOG_HERTZ == 0:
            cloudlog.warning("GPS信号丢失",
                           duration=f"{self.lost_signal_count/LOG_HERTZ}秒",
                           total_lost=self.lost_signal_count)
        return
      # 记录GPS数据
      self.logs.append([
        gps.unixTimestampMillis * 0.001,
        gps.latitude,
        gps.longitude,
        gps.altitude
      ])
      self.log_count += 1
      self.lost_signal_count = 0

      # 检查是否需要暂停
      if not self.pause and gps.speed < MIN_SPEED_THRESHOLD:
        #cloudlog.info("记录暂停 - 车辆停止", log_dir=GPX_ERRORLOGS_PATH)
        self.pause = True

    except Exception as e:
      cloudlog.exception(f"GPS数据记录错误: {str(e)}")

  def write_log(self, force=False):
    if self.log_count == 0:
      return

    should_write = force or \
                   self.log_count >= LOGS_PER_FILE or \
                   self.lost_signal_count >= LOST_SIGNAL_COUNT_MAX

    if should_write:
      try:
        self._write_gpx()
        #cloudlog.info(f"已写入 {self.log_count} 个点到GPX文件", log_dir=GPX_ERRORLOGS_PATH)
        self._reset_state()
      except Exception as e:
        cloudlog.exception(f"GPX文件写入错误: {str(e)}")

  def _reset_state(self):
    self.lost_signal_count = 0
    self.log_count = 0
    self.logs.clear()
    self.started_time = datetime.datetime.utcnow().isoformat()

  def _write_gpx(self):
    if len(self.logs) <= 1:
      return

    filename = datetime.datetime.now().strftime("%Y-%m-%d--%H-%M-%S.gpx")
    filepath = Path(GPX_LOG_PATH) / filename

    try:
      with open(filepath, 'w', encoding='utf-8') as f:
        f.write(self._generate_gpx_content())
    except Exception as e:
      cloudlog.exception(f"写入GPX文件 {filename} 失败: {str(e)}")
      raise

  def _generate_gpx_content(self):
    gpx_lines = [
      '<?xml version="1.0" encoding="utf-8" standalone="yes"?>',
      '<gpx version="1.1" creator="dragonpilot" xmlns="http://www.topografix.com/GPX/1/1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">',
      '<trk>',
      f'<name>{self.started_time}</name>',
      '<trkseg>'
    ]

    for point in self.logs:
      gpx_lines.append(self._format_trackpoint(*point))

    gpx_lines.extend(['</trkseg>', '</trk>', '</gpx>'])
    return '\n'.join(gpx_lines)

  def _format_trackpoint(self, timestamp, lat, lon, alt):
    time_str = datetime.datetime.utcfromtimestamp(timestamp).isoformat()
    return f'<trkpt lat="{lat}" lon="{lon}">\n' \
           f'<time>{time_str}</time>\n' \
           f'<ele>{alt}</ele>\n' \
           f'</trkpt>'

def gpxd_thread(sm=None):
  set_core_affinity([1,])
  set_realtime_priority(1)

  if sm is None:
    sm = messaging.SubMaster(['gpsLocationExternal'])

  try:
    gpxd = GpxD()
    #cloudlog.info("GPX记录器启动", log_dir=GPX_ERRORLOGS_PATH)

    while True:
      sm.update(1000)
      gpxd.log(sm)
      gpxd.write_log()
  except Exception as e:
    cloudlog.exception("GPX记录器异常退出")

def main(sm=None, pm=None):
  gpxd_thread(sm)

if __name__ == "__main__":
  main()
