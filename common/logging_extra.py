import io
import os
import sys
import copy
import json
import time
import uuid
import logging
import traceback
from threading import local
from collections import OrderedDict
from contextlib import contextmanager
import inspect
from logging.handlers import BaseRotatingHandler
from pathlib import Path
LOG_TIMESTAMPS = "LOG_TIMESTAMPS" in os.environ


def get_boot_time():
  """获取系统启动时间作为会话ID"""
  try:
    with open('/proc/uptime', 'r') as f:
      uptime_seconds = float(f.readline().split()[0])
    boot_time = int(time.time() - uptime_seconds)
    return f"{boot_time:x}"  # 转换为16进制
  except Exception:
    # 如果无法获取启动时间，则使用进程启动时间
    return f"{int(time.time()):x}"

def json_handler(obj):
  # if isinstance(obj, (datetime.date, datetime.time)):
  #   return obj.isoformat()
  return repr(obj)

def json_robust_dumps(obj):
  return json.dumps(obj, default=json_handler, ensure_ascii=False)

class NiceOrderedDict(OrderedDict):
  def __str__(self):
    return json_robust_dumps(self)
  def set_module_name(self, name):
    self.bind_global(module=name)

  def format(self, record):
    if self.swaglogger is None:
      raise Exception("must set swaglogger before calling format()")

    # 基本信息
    time_str = self.formatTime(record)
    level = record.levelname
    location = f"{record.filename}:{record.lineno}"
    func_name = record.funcName

    # 消息内容
    if isinstance(record.msg, dict):
      msg = str(record.msg)
    else:
      try:
        msg = record.getMessage()
      except (ValueError, TypeError):
        msg = str([record.msg] + record.args)

    # 异常信息
    error_info = ""
    if record.exc_info:
      error_info = f" | Error: {self.formatException(record.exc_info).replace(chr(10), ' ')}"

    # 上下文信息
    ctx = self.swaglogger.get_ctx()
    ctx_str = f" | Context: {ctx}" if ctx else ""

    # 组合成单行日志
    log_line = f"[{time_str}] [{level}] [{location}] [{func_name}] {msg}{error_info}{ctx_str}"
    return log_line

  def format_dict(self, record):
    # 为了保持兼容性，返回格式化后的字符串
    return self.format(record)


class SwagFormatter(logging.Formatter):
  def __init__(self, swaglogger=None):
    logging.Formatter.__init__(self)
    self.swaglogger = swaglogger

class SwagLogFileFormatter(SwagFormatter):
  def format(self, record):
    if isinstance(record, str):
      return record

    # 统一日志格式
    if self.swaglogger is None:
      return super().format(record)

    # 基本信息
    time_str = time.strftime('%Y-%m-%d %H:%M:%S')
    level = record.levelname
    module = self.swaglogger.get_ctx().get('module', 'unknown')

    # 消息内容
    if isinstance(record.msg, dict):
      msg = str(record.msg)
    else:
      try:
        msg = record.getMessage()
      except (ValueError, TypeError):
        msg = str([record.msg] + record.args)

    # 异常信息
    error_info = ""
    if record.exc_info:
      error_info = f" | Error: {self.formatException(record.exc_info).replace(chr(10), ' ')}"

    # 统一格式: 时间戳 | 日志级别 | 模块名称 | 日志消息
    return f"{time_str} | {level:7s} | {module:15s} | {msg}{error_info}"

class SwagErrorFilter(logging.Filter):
  def filter(self, record):
    return record.levelno < logging.ERROR

def _tmpfunc():
  return 0

def _srcfile():
  return os.path.normcase(_tmpfunc.__code__.co_filename)

class SwaglogRotatingFileHandler(BaseRotatingHandler):
    def __init__(self, base_filename, session_id=None, module_name=None, interval=300, max_bytes=512*1024, backup_count=500, encoding=None, error_log=False):
        self.base_filename = base_filename
        self.session_id = session_id or get_boot_time()
        self.module_name = module_name
        self.interval = interval
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.error_log = error_log
        self.log_files = self.get_existing_logfiles()
        log_indexes = [f.split(".")[-1] for f in self.log_files]
        self.last_file_idx = max([int(i) for i in log_indexes if i.isdigit()] or [-1])
        self.last_rollover = time.monotonic()
        self._current_size = 0
        self._file_created = False
        super().__init__(base_filename, mode="a", encoding=encoding, delay=True)

    def shouldRollover(self, record):
        if not self.stream:
            return False

        try:
            msg_size = len(self.format(record).encode(self.encoding or 'utf-8'))
            self._current_size += msg_size

            size_exceeded = self.max_bytes > 0 and self._current_size >= self.max_bytes
            time_exceeded = self.interval > 0 and self.last_rollover + self.interval <= time.monotonic()

            if time_exceeded and self._current_size < 1024:
                self.last_rollover = time.monotonic()
                return False

            return size_exceeded or time_exceeded
        except (AttributeError, OSError):
            return True

    def emit(self, record):
        try:
            # 检查是否需要创建文件
            if not self._file_created:
                if not self.error_log or (self.error_log and record.levelno >= logging.ERROR):
                    self._file_created = True
                else:
                    return

            if self.shouldRollover(record):
                self.doRollover()
            if not self.stream:
                self.stream = self._open()
            msg = self.format(record)
            self.stream.write(msg + self.terminator)
            self.stream.flush()
        except Exception as e:
            self.handleError(record)

    def _open(self):
        self.last_rollover = time.monotonic()
        self.last_file_idx += 1
        date_str = time.strftime("%Y%m%d_%H%M%S")

        base_name = self.module_name or os.path.basename(self.base_filename)
        if self.error_log:
            base_name = f"{base_name}_error"

        next_filename = os.path.join(
            os.path.dirname(self.base_filename),
            f"{base_name}.{date_str}.{self.session_id}.{self.last_file_idx:03d}.log"
        )

        self._current_size = 0

        while os.path.exists(next_filename):
            self.last_file_idx += 1
            next_filename = os.path.join(
                os.path.dirname(self.base_filename),
                f"{base_name}.{date_str}.{self.session_id}.{self.last_file_idx:03d}.log"
            )

        # 确保目录存在
        os.makedirs(os.path.dirname(next_filename), exist_ok=True)

        stream = open(next_filename, self.mode, encoding=self.encoding)
        self.log_files.insert(0, next_filename)
        return stream

    def get_existing_logfiles(self):
        log_files = list()
        base_dir = os.path.dirname(self.base_filename)
        if os.path.exists(base_dir):
            for fn in os.listdir(base_dir):
                fp = os.path.join(base_dir, fn)
                if fp.startswith(self.base_filename) and os.path.isfile(fp):
                    log_files.append(fp)
        return sorted(log_files)

    def doRollover(self):
        if self.stream:
            try:
                current_size = self.stream.tell()
                if current_size == 0 or current_size < 1024:
                    self.last_rollover = time.monotonic()
                    self._current_size = current_size
                    return

                self.stream.close()
            except Exception:
                pass

        self.stream = self._open()

        if self.backup_count > 0:
            while len(self.log_files) > self.backup_count:
                to_delete = self.log_files.pop()
                try:
                    if os.path.exists(to_delete):
                        if os.path.getsize(to_delete) == 0 or os.path.getsize(to_delete) < 1024:
                            os.remove(to_delete)
                        else:
                            os.remove(to_delete)
                except Exception:
                    continue

def get_custom_file_handler(log_dir, module_name=None):
  try:
    # 如果没有指定模块名，尝试获取调用者的模块名
    if module_name is None:
      frame = inspect.currentframe()
      caller_frame = frame.f_back
      if caller_frame:
        module = inspect.getmodule(caller_frame)
        if module:
          module_name = module.__name__.split('.')[-1]
        else:
          # 尝试从调用栈更深处获取模块名
          caller_frame = caller_frame.f_back
          if caller_frame:
            module = inspect.getmodule(caller_frame)
            if module:
              module_name = module.__name__.split('.')[-1]

    # 如果仍然无法获取模块名，使用默认值
    if not module_name:
      module_name = "unknown"

    # 确保日志目录存在
    Path(log_dir).mkdir(parents=True, exist_ok=True)

    # 使用进程ID作为会话ID的一部分，确保唯一性
    session_id = get_boot_time()

    # 创建基础文件名
    base_filename = os.path.join(log_dir, module_name)

    # 创建普通日志处理器
    handler = SwaglogRotatingFileHandler(
      base_filename,
      session_id=session_id,
      module_name=module_name,
      max_bytes=512*1024,    # 增加到512KB
      interval=300,           # 增加到5分钟
      backup_count=500        # 减少到100个备份
    )

    # 只在需要时创建错误日志处理器
    error_handler = None
    # if os.path.exists(log_dir):  # 如果目录已经存在，才创建错误日志处理器
    #   error_handler = SwaglogRotatingFileHandler(
    #     base_filename,
    #     session_id=session_id,
    #     module_name=module_name,
    #     max_bytes=156*1024,
    #     interval=180,
    #     backup_count=1000,
    #     error_log=True
    #   )

    # 确保至少有一个处理器有效
    if not handler:
      return None

    return handler, error_handler
  except Exception as e:
    print(f"创建自定义文件处理器失败: {str(e)}")
    return None


class SwagLogger(logging.Logger):
  def __init__(self):
    logging.Logger.__init__(self, "swaglog")
    self.global_ctx = {}
    self.log_local = local()
    self.log_local.ctx = {}
    self._custom_handlers = {}
    self._error_handlers = {}  # 新增：存储错误日志处理器

  def add_custom_handler(self, log_dir, module_name=None):
    handler_key = f"{log_dir}:{module_name or self.get_ctx().get('module', 'unknown')}"

    if handler_key not in self._custom_handlers:
      handlers = get_custom_file_handler(log_dir, module_name)
      if handlers:
        handler, error_handler = handlers

        # 设置普通日志处理器
        handler.setFormatter(SwagLogFileFormatter(self))
        handler.addFilter(SwagErrorFilter())
        handler.setLevel(logging.DEBUG)
        self.addHandler(handler)
        self._custom_handlers[handler_key] = handler

        # 只在有错误处理器时添加
        if error_handler:
          error_handler.setFormatter(SwagLogFileFormatter(self))
          error_handler.setLevel(logging.ERROR)
          self.addHandler(error_handler)
          self._error_handlers[handler_key] = error_handler
        return True
    return False

  def remove_custom_handler(self, log_dir, module_name=None):
    """移除自定义目录的日志处理器"""
    handler_key = f"{log_dir}:{module_name or self.get_ctx().get('module', 'unknown')}"

    if handler_key in self._custom_handlers:
      handler = self._custom_handlers.pop(handler_key)
      self.removeHandler(handler)
      handler.close()

    if handler_key in self._error_handlers:
      error_handler = self._error_handlers.pop(handler_key)
      self.removeHandler(error_handler)
      error_handler.close()

  def _log(self, level, msg, args, exc_info=None, extra=None, stack_info=False, stacklevel=1, **kwargs):
    """重写 _log 方法以处理额外的关键字参数"""
    # 提取自定义参数
    log_dir = kwargs.pop('log_dir', None)
    module_name = kwargs.pop('module_name', None)

    # 保存原始上下文
    old_ctx = None
    if module_name:
      old_ctx = self.get_ctx().copy()
      self.bind(module=module_name)

    try:
      # 检查消息是否为空
      if not msg and not args and not exc_info:
        return

      # 检查是否需要添加自定义日志处理器
      if log_dir:
        self.add_custom_handler(log_dir, module_name)

      # 将剩余的关键字参数添加到上下文中
      if kwargs:
        self.bind(**kwargs)

      # 调用父类的 _log 方法
      super()._log(level, msg, args, exc_info, extra, stack_info, stacklevel)

    finally:
      # 恢复原始上下文
      if module_name and old_ctx:
        self.log_local.ctx.clear()
        self.bind(**old_ctx)

  def local_ctx(self):
    try:
      return self.log_local.ctx
    except AttributeError:
      self.log_local.ctx = {}
      return self.log_local.ctx

  def get_ctx(self):
    return dict(self.local_ctx(), **self.global_ctx)

  @contextmanager
  def ctx(self, **kwargs):
    old_ctx = self.local_ctx()
    self.log_local.ctx = copy.copy(old_ctx) or {}
    self.log_local.ctx.update(kwargs)
    try:
      yield
    finally:
      self.log_local.ctx = old_ctx

  def bind(self, **kwargs):
    self.local_ctx().update(kwargs)

  def bind_global(self, **kwargs):
    self.global_ctx.update(kwargs)

  def event(self, event, *args, **kwargs):
    evt = NiceOrderedDict()
    evt['event'] = event
    if args:
      evt['args'] = args
    evt.update(kwargs)
    if 'error' in kwargs:
      self.error(evt)
    elif 'debug' in kwargs:
      self.debug(evt)
    else:
      self.info(evt)

  def timestamp(self, event_name):
    if LOG_TIMESTAMPS:
      t = time.monotonic()
      tstp = NiceOrderedDict()
      tstp['timestamp'] = NiceOrderedDict()
      tstp['timestamp']["event"] = event_name
      tstp['timestamp']["time"] = t*1e9
      self.debug(tstp)

  def findCaller(self, stack_info=False, stacklevel=1):
    f = sys._getframe(3)
    if f is not None:
      f = f.f_back
    orig_f = f
    while f and stacklevel > 1:
      f = f.f_back
      stacklevel -= 1
    if not f:
      f = orig_f
    rv = "(unknown file)", 0, "(unknown function)", None
    while hasattr(f, "f_code"):
      co = f.f_code
      filename = os.path.normcase(co.co_filename)
      if filename == _srcfile:
        f = f.f_back
        continue
      sinfo = None
      if stack_info:
        sio = io.StringIO()
        sio.write('Stack (most recent call last):\n')
        traceback.print_stack(f, file=sio)
        sinfo = sio.getvalue()
        if sinfo[-1] == '\n':
          sinfo = sinfo[:-1]
        sio.close()
      rv = (co.co_filename, f.f_lineno, co.co_name, sinfo)
      break
    return rv

if __name__ == "__main__":
  log = SwagLogger()

  stdout_handler = logging.StreamHandler(sys.stdout)
  stdout_handler.setLevel(logging.INFO)
  stdout_handler.addFilter(SwagErrorFilter())
  log.addHandler(stdout_handler)

  stderr_handler = logging.StreamHandler(sys.stderr)
  stderr_handler.setLevel(logging.ERROR)
  log.addHandler(stderr_handler)

  stdout_handler.setFormatter(SwagFormatter(log))
  stderr_handler.setFormatter(SwagFormatter(log))

  log.info("测试日志 %s", "参数")
  log.info({'数据': 1})
  log.warning("警告信息")
  log.error("错误信息")
  log.critical("严重错误")
  log.event("测试事件", x="y")

  with log.ctx():
    log.bind(user="测试用户")
    log.info("带上下文的日志")
    log.warning("带上下文的警告")
    log.error("带上下文的错误")
    log.critical("带上下文的严重错误")
