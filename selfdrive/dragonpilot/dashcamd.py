import os
import time
import datetime
import subprocess
from pathlib import Path
from openpilot.common.swaglog import cloudlog

DASHCAM_VIDEOS_PATH = '/data/media/0/dashcam/'
#DASHCAM_LOGS_PATH = '/data/media/0/c2_logs/dashcam_logs/'

# 视频质量预设
QUALITY_PRESETS = {
  "low": {
    "bitrate": 2000000,      # 2Mbps
    "resolution": "1280x720"  # 降低分辨率
  },
  "medium": {
    "bitrate": 4000000,      # 4Mbps
    "resolution": None        # 保持原始分辨率
  },
  "high": {
    "bitrate": 8000000,      # 8Mbps
    "resolution": None        # 保持原始分辨率
  }
}

class Dashcamd:
  def __init__(self, config=None):
    # 默认配置
    self.config = {
      'enabled': False,
      'quality': "medium",
      'duration': 180,
      'kept_hours': 15,
    }

    self.recording = False
    self.free_space = 1.
    self.current_process = None
    self.session_id = int(time.monotonic() * 1000)
    self.file_counter = 0

    # 设置模块名称
    cloudlog.bind_global(module='dashcamd')

    # 确保日志和视频目录存在
    #Path(DASHCAM_LOGS_PATH).mkdir(parents=True, exist_ok=True)
    Path(DASHCAM_VIDEOS_PATH).mkdir(parents=True, exist_ok=True)

    # 添加会话信息到日志上下文
    cloudlog.bind(session_id=self.session_id)

    # 更新配置
    if config:
      self.update_config(config)

  def update_config(self, config):
    self.config.update(config)
    #cloudlog.debug(f"配置更新: config={self.config}, quality={self.config['quality']}")

    quality = self.config['quality']
    if quality not in QUALITY_PRESETS:
      quality = "medium"

    self.quality_settings = QUALITY_PRESETS[quality]
    self._update_storage_limits()

  def _update_storage_limits(self):
    """根据配置更新存储限制"""
    # 限制录制时长在1-3分钟之间
    self.DASHCAM_DURATION = max(60, min(180, self.config['duration']))
    self.DASHCAM_BIT_RATES = self.quality_settings["bitrate"]
    self.DASHCAM_MAX_SIZE_PER_FILE = self.DASHCAM_BIT_RATES / 8 * self.DASHCAM_DURATION
    self.DASHCAM_FREESPACE_LIMIT = 15
    kept_hours = max(1, min(72, self.config['kept_hours']))  # 限制在1-72小时之间
    self.DASHCAM_KEPT_MIN_SIZE = self.DASHCAM_MAX_SIZE_PER_FILE * (kept_hours * 60 * 60 / self.DASHCAM_DURATION)

  def start_recording(self, free_space):
    """开始循环录制"""
    self.free_space = free_space

    if self.recording:
      return

    self.recording = True
    #cloudlog.info("开始循环录制")

    # 启动录制循环
    self._record_next_file()

  def stop_recording(self):
    """停止录制"""
    if not self.recording:
      return

    self.recording = False
    self._stop_current_recording()
    #cloudlog.info("停止录制")

  def _stop_current_recording(self):
    """停止当前录制进程"""
    try:
      if self.current_process:
        self.current_process.terminate()
        self.current_process.wait(timeout=2)
      else:
        subprocess.run(['killall', '-SIGINT', 'screenrecord'], check=False)
    except Exception as e:
      cloudlog.error(f"停止录制出错: {str(e)}")
    finally:
      self.current_process = None

  def _record_next_file(self):
    """录制下一个文件"""
    if not self.recording:
      return

    # 先清理空间
    self._clean_up_space()

    try:
      now = datetime.datetime.now()
      file_name = f"{now.strftime('%Y%m%d_%H%M%S')}_{self.session_id}_f{self.file_counter:04d}_{self.config['quality']}"
      self.file_counter += 1
      full_path = os.path.join(DASHCAM_VIDEOS_PATH, f"{file_name}.mp4")

      # 构建录制命令
      cmd = ["screenrecord", "--bit-rate", str(self.DASHCAM_BIT_RATES),
             "--time-limit", str(self.DASHCAM_DURATION)]
      if self.quality_settings["resolution"]:
        cmd.extend(["--size", self.quality_settings["resolution"]])
      cmd.append(full_path)

      env = os.environ.copy()
      env["LD_LIBRARY_PATH"] = ""

      # 启动录制进程
      self.current_process = subprocess.Popen(cmd, env=env)

      quality_info = f"质量: {self.config['quality']}, 比特率: {self.DASHCAM_BIT_RATES}"
      if self.quality_settings["resolution"]:
        quality_info += f", 分辨率: {self.quality_settings['resolution']}"
      #cloudlog.info(f"开始录制文件: {full_path}, {quality_info}, 时长: {self.DASHCAM_DURATION}秒")

      # 启动监控线程，等待当前录制完成后继续下一个
      import threading
      threading.Thread(target=self._wait_and_record_next, daemon=True).start()

    except Exception as e:
      cloudlog.error(f"开始录制出错: {str(e)}")
      # 如果出错，稍后重试
      import threading
      threading.Timer(5.0, self._record_next_file).start()

  def _wait_and_record_next(self):
    """等待当前录制完成并开始下一个"""
    if self.current_process:
      self.current_process.wait()

    # 如果仍在录制状态，则继续下一个文件
    if self.recording:
      self._record_next_file()

  def _clean_up_space(self):
    """清理空间，删除最旧的文件"""
    try:
      # 检查是否需要清理空间
      if (self.free_space < self.DASHCAM_FREESPACE_LIMIT) or (self._get_used_space() > self.DASHCAM_KEPT_MIN_SIZE):
        files = []
        for f in os.listdir(DASHCAM_VIDEOS_PATH):
          if not f.endswith('.mp4'):
            continue
          full_path = os.path.join(DASHCAM_VIDEOS_PATH, f)
          if not os.path.isfile(full_path) or os.path.getsize(full_path) == 0:
            continue

          # 尝试从文件名解析信息
          try:
            parts = f.split('_')
            if len(parts) >= 4 and parts[0].startswith('s') and parts[1].startswith('f'):
              session_id = int(parts[0][1:])
              file_num = int(parts[1][1:])
              files.append((full_path, session_id, file_num))
            else:
              files.append((full_path, 0, os.path.getmtime(full_path)))
          except:
            files.append((full_path, 0, os.path.getmtime(full_path)))

        # 按会话ID和文件编号排序，删除最旧的
        if files:
          files.sort(key=lambda x: (x[1], x[2]))
          os.remove(files[0][0])
          #cloudlog.info(f"已删除旧文件: {files[0][0]}")
    except Exception as e:
      cloudlog.error(f"清理空间出错: {str(e)}")

  def _get_used_space(self):
    """获取已使用的空间大小"""
    try:
      return sum(
        os.path.getsize(os.path.join(DASHCAM_VIDEOS_PATH, f))
        for f in os.listdir(DASHCAM_VIDEOS_PATH)
        if os.path.isfile(os.path.join(DASHCAM_VIDEOS_PATH, f))
      )
    except Exception as e:
      cloudlog.error(f"计算空间使用出错: {str(e)}")
      return 0

  def run(self, started=False, free_space=100.0):
    """运行行车记录仪
    Args:
        started (bool): 是否开始录制
        free_space (float): 可用空间百分比
    """
    try:
      if started:
        self.start_recording(free_space)
      else:
        self.stop_recording()
    except Exception as e:
      cloudlog.exception(f"行车记录仪运行异常: {str(e)}")

  def restart_recording(self):
    """重新开始录制"""
    if self.recording and self.current_process is not None:
      self.stop_recording()
      time.sleep(1)  # 等待进程完全停止
      self.start_recording(self.free_space)
