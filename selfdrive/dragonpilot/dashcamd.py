import os
import threading
import time
import datetime
import subprocess
from pathlib import Path
from openpilot.common.swaglog import cloudlog

DASHCAM_VIDEOS_PATH = '/data/media/0/dashcam/'

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

# 数值到质量字符串的映射
QUALITY_MAP = {
    0: "low",
    1: "medium", 
    2: "high"
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
    self.cleanup_lock = threading.Lock()
    self.last_clean_time = 0
    self.CLEAN_INTERVAL = 3600  # 1小时强制清理一次
    self.video_dir = DASHCAM_VIDEOS_PATH

    # 初始化缺失的属性
    self.quality_settings = QUALITY_PRESETS["medium"]  # 默认中等质量
    self._update_storage_limits()  # 初始化存储限制相关属性

    Path(self.video_dir).mkdir(parents=True, exist_ok=True)  # 确保目录存在

  def update_config(self, config):
      self.config.update(config)
      
      # 处理quality参数
      quality = self.config['quality']
      try:
          # 如果是数值，转换为字符串
          if isinstance(quality, int) or (isinstance(quality, str) and quality.isdigit()):
              quality = int(quality)
              quality = QUALITY_MAP.get(quality, "medium")
      except (ValueError, TypeError):
          quality = "medium"  # 默认中等质量
  
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
    self.DASHCAM_FREESPACE_LIMIT = 30  # 改为30%空间预留
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
      try:
        if self.current_process and self.current_process.poll() is None:  # 添加进程状态检查
          self.current_process.terminate()
          try:
            self.current_process.wait(timeout=2)
          except subprocess.TimeoutExpired:
            self.current_process.kill()
            self.current_process.wait()
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
      # 获取质量预设的英文名称
      quality_name = self.quality_settings.get('name', self.config['quality'])
      file_name = f"dashcam_{now.strftime('%Y%m%d_%H%M%S')}_{self.session_id}_f{self.file_counter:04d}_{quality_name}"
      self.file_counter += 1
      full_path = os.path.join(self.video_dir, f"{file_name}.mp4")  # 使用self.video_dir替代DASHCAM_VIDEOS_PATH

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
      # 可以改为使用质量名称
      quality_info = f"质量: {quality_name}, 比特率: {self.DASHCAM_BIT_RATES//1000000}Mbps"  # 更易读的比特率显示
      if self.quality_settings["resolution"]:
        quality_info += f", 分辨率: {self.quality_settings['resolution'] or '原始分辨率'}"
      #cloudlog.info(f"开始录制文件: {full_path}, {quality_info}, 时长: {self.DASHCAM_DURATION}秒")

      # 启动监控线程，等待当前录制完成后继续下一个
      threading.Thread(target=self._wait_and_record_next, daemon=True).start()

    except Exception as e:
      cloudlog.error(f"开始录制出错: {str(e)}")
      threading.Timer(5.0, self._record_next_file).start()

  def _wait_and_record_next(self):
    """等待当前录制完成并开始下一个"""
    if self.current_process:
      self.current_process.wait()

    # 如果仍在录制状态，则继续下一个文件
    if self.recording:
      self._record_next_file()

  def _clean_up_space(self):
      with self.cleanup_lock:
          try:
              # 添加定期清理条件
              need_clean = (self.free_space < self.DASHCAM_FREESPACE_LIMIT or 
                           self._get_used_space() > self.DASHCAM_KEPT_MIN_SIZE or
                           time.time() - self.last_clean_time > self.CLEAN_INTERVAL)
              
              if need_clean:
                  files = []
                  for f in os.listdir(self.video_dir):  # 改为self.video_dir
                      full_path = os.path.join(self.video_dir, f)  # 改为self.video_dir
                      if not os.path.isfile(full_path):
                          continue

                      try:
                          # 一次性获取文件大小和修改时间
                          stat = os.stat(full_path)
                          if stat.st_size == 0:  # 空文件直接删除
                              os.remove(full_path)
                              continue
                          files.append((full_path, stat.st_mtime, stat.st_size))  # 保存路径、修改时间和大小
                      except Exception:
                          continue

                  # 按修改时间排序，删除最旧的
                  if files:
                      files.sort(key=lambda x: x[1])  # 按修改时间排序
                      space_needed = self.DASHCAM_MAX_SIZE_PER_FILE * 5
                      space_freed = 0
                      for file_info in files:
                          if space_freed >= space_needed:
                              break
                          try:
                              os.remove(file_info[0])
                              space_freed += file_info[2]  # 使用预存的文件大小
                              cloudlog.info(f"已删除旧文件: {file_info[0]}") 
                          except Exception as e:
                              cloudlog.error(f"删除文件失败: {str(e)}")
                              continue
          except Exception as e:
              cloudlog.error(f"清理空间出错: {str(e)}")
      self.last_clean_time = time.time()

  def _get_used_space(self):
    """获取已使用的空间大小"""
    try:
      return sum(
        os.path.getsize(os.path.join(self.video_dir, f))  # 改为self.video_dir
        for f in os.listdir(self.video_dir)  # 改为self.video_dir
        if os.path.isfile(os.path.join(self.video_dir, f))  # 改为self.video_dir
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


  def cleanup(self):
      """清理所有资源"""
      self.stop_recording()
      if hasattr(self, 'cleanup_lock'):
          try:
              self.cleanup_lock.acquire()
              # 可以添加其他资源清理逻辑
          finally:
              self.cleanup_lock.release()
              del self.cleanup_lock
