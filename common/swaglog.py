'''
rick:
this file is identical to system/swaglog.py, put a copy here to reduce code changes when updating from op master branch.
'''
import logging
import os
import warnings
import time
import datetime
from pathlib import Path
import zmq
from openpilot.common.logging_extra import SwagLogger, SwagFormatter, SwagLogFileFormatter, SwaglogRotatingFileHandler,SwagErrorFilter, get_custom_file_handler,get_boot_time
from openpilot.system.hardware import PC
from openpilot.common.params import Params

if PC:
  SWAGLOG_DIR = os.path.join(str(Path.home()), ".comma", "log")
else:
  MEDIA_PATH = "/data/media/0/c2_logs/swaglog"
  DEFAULT_PATH = "/data/log/"
  SWAGLOG_DIR = MEDIA_PATH if os.path.exists("/data/media/0") else DEFAULT_PATH

def get_file_handler():
  try:
    Path(SWAGLOG_DIR).mkdir(parents=True, exist_ok=True)
    base_filename = os.path.join(SWAGLOG_DIR, "swaglog")
    handler = SwaglogRotatingFileHandler(
      base_filename,
      max_bytes=512*1024,    # 增加到1MB
      interval=300,           # 5分钟
      backup_count=500        # 减少备份数量
    )
    return handler
  except Exception as e:
    print(f"Error creating file handler: {str(e)}")
    return None

class UnixDomainSocketHandler(logging.Handler):
  def __init__(self, formatter):
    logging.Handler.__init__(self)
    self.setFormatter(formatter)
    self.pid = None
    self.zctx = None
    self.sock = None

  def __del__(self):
    if self.sock is not None:
      self.sock.close()
    if self.zctx is not None:
      self.zctx.term()

  def connect(self):
    self.zctx = zmq.Context()
    self.sock = self.zctx.socket(zmq.PUSH)
    self.sock.setsockopt(zmq.LINGER, 10)
    self.sock.connect("ipc:///tmp/logmessage")
    self.pid = os.getpid()

  def emit(self, record):
    if os.getpid() != self.pid:
      warnings.filterwarnings("ignore", category=ResourceWarning, message="unclosed.*<zmq.*>")
      self.connect()

    msg = self.format(record).rstrip('\n')
    try:
      s = chr(record.levelno)+msg
      self.sock.send(s.encode('utf8'), zmq.NOBLOCK)
    except zmq.error.Again:
      pass

def add_file_handler(log, module_name=None):
  # 如果没有指定模块名，尝试获取调用者的模块名
  if module_name is None:
    import inspect
    frame = inspect.currentframe().f_back
    module = inspect.getmodule(frame)
    if module:
      module_name = module.__name__.split('.')[-1]

  handler = get_file_handler()
  if handler:
    handler.setFormatter(SwagLogFileFormatter(log))
    # 如果有模块名，设置到处理器中
    if module_name and hasattr(handler, 'module_name'):
      handler.module_name = module_name
    log.addHandler(handler)

# 添加自定义日志目录支持
def log_to_custom_dir(message, level, module_name, log_dir):
  """将日志以标准格式写入自定义目录"""
  try:
    # 确保日志目录存在
    Path(log_dir).mkdir(parents=True, exist_ok=True)

    # 生成会话ID (使用进程ID作为会话ID的一部分)
    session_id = get_boot_time()

    # 格式化时间戳
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    date_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # 修改文件名格式，保持一致性
    log_file = os.path.join(log_dir, f"{module_name}.{date_str}.{session_id}.{file_idx:03d}.log")

    # 检查文件是否存在，如果存在则增加序号
    file_idx = 1
    while os.path.exists(log_file):
      file_idx += 1
      log_file = os.path.join(log_dir, f"{module_name}.{date_str}.{session_id}.{file_idx:03d}.log")

    # 统一日志格式
    formatted_message = f"{timestamp} | {level:7s} | {module_name:15s} | {message}"

    # 写入日志
    with open(log_file, 'a', encoding='utf-8') as f:
      f.write(formatted_message + "\n")

  except Exception as e:
    print(f"写入自定义日志失败: {str(e)}")

cloudlog = log = SwagLogger()
log.setLevel(logging.DEBUG)

# 扩展原始日志方法
original_debug = cloudlog.debug
original_info = cloudlog.info
original_warning = cloudlog.warning
original_error = cloudlog.error

# 包装日志方法以支持自定义目录
def wrap_log_method(original_method, level_name):
  def wrapped_method(msg, *args, **kwargs):
    # 提取自定义参数
    log_dir = kwargs.pop('log_dir', None)
    module_name = kwargs.pop('module_name', None)

    # 检查消息是否为空
    if not msg and not args:
      return None

    # 保存原始上下文
    old_ctx = None
    if module_name:
      old_ctx = cloudlog.get_ctx().copy()
      cloudlog.bind(module=module_name)

    try:
      # 如果指定了日志目录，添加自定义处理器
      if log_dir:
        current_module = module_name or cloudlog.get_ctx().get('module', 'unknown')
        handler_key = f"{log_dir}:{current_module}"

        # 添加或获取处理器
        if not hasattr(cloudlog, '_custom_handlers'):
          cloudlog._custom_handlers = {}
          cloudlog._error_handlers = {}

        if handler_key not in cloudlog._custom_handlers:
          handlers = get_custom_file_handler(log_dir, current_module)
          if handlers:
            handler, error_handler = handlers

            # 设置普通日志处理器
            if level_name != "ERROR":
              handler.setFormatter(SwagLogFileFormatter(cloudlog))
              handler.addFilter(SwagErrorFilter())
              cloudlog.addHandler(handler)
              cloudlog._custom_handlers[handler_key] = handler

            # 设置错误日志处理器
            if level_name == "ERROR" and error_handler:
              error_handler.setFormatter(SwagLogFileFormatter(cloudlog))
              error_handler.setLevel(logging.ERROR)
              cloudlog.addHandler(error_handler)
              cloudlog._error_handlers[handler_key] = error_handler

      # 调用原始方法
      return original_method(msg, *args, **kwargs)

    finally:
      # 恢复原始上下文
      if module_name and old_ctx:
        cloudlog.log_local.ctx = old_ctx

  return wrapped_method

# 替换原始日志方法
cloudlog.debug = wrap_log_method(original_debug, "DEBUG")
cloudlog.info = wrap_log_method(original_info, "INFO")
cloudlog.warning = wrap_log_method(original_warning, "WARNING")
cloudlog.error = wrap_log_method(original_error, "ERROR")

outhandler = logging.StreamHandler()
# 优先使用 dp_log_level 参数，如果没有则使用环境变量
params = Params()
dp_log_level = params.get("dp_log_level", encoding='utf8')

# 数字映射到日志等级
if dp_log_level is not None:
  try:
    level_map = {
      "0": "warning",
      "1": "info",
      "2": "debug"
    }
    print_level = level_map.get(dp_log_level, "warning")
  except Exception:
    print_level = "warning"
else:
  print_level = os.environ.get('LOGPRINT', 'warning')
if print_level == 'debug':
  outhandler.setLevel(logging.DEBUG)
elif print_level == 'info':
  outhandler.setLevel(logging.INFO)
elif print_level == 'warning':
  outhandler.setLevel(logging.WARNING)

log.addHandler(outhandler)
log.addHandler(UnixDomainSocketHandler(SwagFormatter(log)))
