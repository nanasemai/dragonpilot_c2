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

    # 更新配置
    if config:
      self.update_config(config)

  def update_config(self, config):
      self.config.update(config)
      cloudlog.info(f"Dashcamd update_config - self.config['duration']: {self.config['duration']}") # 添加日志

      # 处理quality参数
      quality = self.config['quality']
      determined_quality_string = "medium" # Default
      try:
          # 如果是数值，转换为字符串
          if isinstance(quality, int) or (isinstance(quality, str) and quality.isdigit()):
              quality_int = int(quality)
              determined_quality_string = QUALITY_MAP.get(quality_int, "medium")
          elif isinstance(quality, str) and quality in QUALITY_PRESETS:
              determined_quality_string = quality
      except (ValueError, TypeError):
          determined_quality_string = "medium"  # 默认中等质量

      if determined_quality_string not in QUALITY_PRESETS:
          determined_quality_string = "medium"

      self.quality_settings = QUALITY_PRESETS[determined_quality_string]
      # Store the determined string quality back into config for consistent access
      self.config['quality'] = determined_quality_string # <-- Add this line
      self._update_storage_limits()

  def _update_storage_limits(self):
      self.DASHCAM_DURATION = self.config['duration']  # 直接使用配置值
      self.DASHCAM_BIT_RATES = self.quality_settings["bitrate"]
      self.DASHCAM_MAX_SIZE_PER_FILE = self.DASHCAM_BIT_RATES / 8 * self.DASHCAM_DURATION
      self.DASHCAM_FREESPACE_LIMIT = 10  # 10%空间预留
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
    # 优化: 确保录制目录存在，防止运行时意外删除
    if not os.path.exists(self.video_dir):
      try:
        Path(self.video_dir).mkdir(parents=True, exist_ok=True)
      except Exception:
        pass

    self._clean_up_space()

    try:
      now = datetime.datetime.now()
      # 获取质量预设的英文名称
      # quality_name = self.quality_settings.get('name', self.config['quality']) # Original line
      quality_name = self.config['quality'] # Use the string stored in config
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

      # quality_info = f"质量: {self.config['quality']}, 比特率: {self.DASHCAM_BIT_RATES}" # Original line
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
        # 1. 获取准确的磁盘空间信息
        try:
          stat = os.statvfs(self.video_dir)
          # available_bytes: 非特权用户可用空间
          available_bytes = stat.f_bavail * stat.f_frsize
          total_bytes = stat.f_blocks * stat.f_frsize
        except OSError:
          # Fallback: 如果 statvfs 失败，使用 factory reset 后的估算或 systemd 传入的值
          total_bytes = self._get_total_space()
          available_bytes = (self.free_space / 100.0) * total_bytes

        # 2. 收集文件列表并计算当前占用 (使用 os.scandir 优化性能)
        files = []
        current_used_space = 0
        try:
          with os.scandir(self.video_dir) as entries:
            for entry in entries:
              if not entry.is_file():
                continue
              
              try:
                # entry.stat() 在 Linux 上通常会被缓存，比 os.stat(path) 更快
                f_stat = entry.stat()
              except OSError:
                continue

              full_path = entry.path

              # 顺便清理 0 字节文件
              if f_stat.st_size == 0:
                try:
                  os.remove(full_path)
                  cloudlog.info(f"已删除空文件: {full_path}")
                except OSError:
                  pass
                continue
              
              files.append((full_path, f_stat.st_mtime, f_stat.st_size))
              current_used_space += f_stat.st_size
        except OSError as e:
           cloudlog.error(f"扫描文件列表出错: {str(e)}")
           return

        # 3. 计算需要释放的空间
        
        # A. 剩余空间策略 (Free Space Policy)
        # 目标: 保持磁盘剩余空间 > DASHCAM_FREESPACE_LIMIT (例如 10%)
        # 这个逻辑优先于保留时长，确保系统不爆满
        target_free_bytes = (self.DASHCAM_FREESPACE_LIMIT / 100.0) * total_bytes
        bytes_to_free_for_space = max(0, target_free_bytes - available_bytes)

        # B. 占用上限/保留时长策略 (Size/Time Policy)
        # 目标: Dashcam 文件夹总大小 < DASHCAM_KEPT_MIN_SIZE
        bytes_to_free_for_limit = max(0, current_used_space - self.DASHCAM_KEPT_MIN_SIZE)

        # 取两者最大值作为清理目标
        total_bytes_to_free = max(bytes_to_free_for_space, bytes_to_free_for_limit)

        current_time = time.time()
        
        # 只有在确实需要清理，或者距离上次清理很久了(虽然这里每次都扫描了)，更新一下时间戳
        # 这里的 CLEAN_INTERVAL 主要是给 systemd 这种外部调用做参考，但这里既然已经进来了，就执行逻辑
        if total_bytes_to_free > 0 or (current_time - self.last_clean_time > self.CLEAN_INTERVAL):
             self.last_clean_time = current_time

        if total_bytes_to_free > 0:
          # 按修改时间排序 (最早的在前)
          files.sort(key=lambda x: x[1])
          
          freed_bytes = 0
          for full_path, mtime, size in files:
            if freed_bytes >= total_bytes_to_free:
              break
            
            try:
              os.remove(full_path)
              freed_bytes += size
              # 更新 current_used_space 以便日志记录 (可选)
              current_used_space -= size
              cloudlog.info(f"已删除旧文件: {os.path.basename(full_path)} (释放: {size/1e6:.1f}MB, 剩余需释放: {(total_bytes_to_free - freed_bytes)/1e6:.1f}MB)")
            except OSError as e:
              cloudlog.error(f"删除文件失败: {str(e)}")

      except Exception as e:
        cloudlog.error(f"清理空间出错: {str(e)}")

  def _get_used_space(self):
    """获取已使用的空间大小 (使用 scandir 优化)"""
    try:
      total_size = 0
      with os.scandir(self.video_dir) as entries:
        for entry in entries:
          if entry.is_file():
            try:
              total_size += entry.stat().st_size
            except OSError:
              pass
      return total_size
    except Exception as e:
      cloudlog.error(f"计算空间使用出错: {str(e)}")
      return 0

  def _get_total_space(self):
      """获取总空间大小 (字节)"""
      try:
          statvfs = os.statvfs(self.video_dir)
          return statvfs.f_blocks * statvfs.f_bsize
      except Exception as e:
          cloudlog.error(f"获取总空间出错: {str(e)}")
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
