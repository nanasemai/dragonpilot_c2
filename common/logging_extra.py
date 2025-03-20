import io
import os
import sys
import copy
import json
import time
import logging
import traceback
from threading import local
from collections import OrderedDict
from contextlib import contextmanager
import inspect
from logging.handlers import BaseRotatingHandler
from pathlib import Path

# 常量定义
LOG_TIMESTAMPS = "LOG_TIMESTAMPS" in os.environ
DEFAULT_ENCODING = 'utf-8'
MIN_FILE_SIZE = 1024  # 1KB
DEFAULT_MAX_BYTES = 512 * 1024  # 512KB
DEFAULT_INTERVAL = 300  # 5分钟
DEFAULT_BACKUP_COUNT = 500

def get_boot_time():
    """获取系统启动时间作为会话ID"""
    try:
        with open('/proc/uptime', 'r') as f:
            uptime_seconds = float(f.readline().split()[0])
        boot_time = int(time.time() - uptime_seconds)
        return f"{boot_time:x}"
    except Exception:
        return f"{int(time.time()):x}"

def json_handler(obj):
    return repr(obj)

def json_robust_dumps(obj):
    return json.dumps(obj, default=json_handler, ensure_ascii=False)

class NiceOrderedDict(OrderedDict):
    def __str__(self):
        return json_robust_dumps(self)

class SwagFormatter(logging.Formatter):
    def __init__(self, swaglogger=None):
        super().__init__()
        self.swaglogger = swaglogger

    def format(self, record):
        """格式化日志记录"""
        if isinstance(record, str):
            return record

        # 基础信息
        time_str = time.strftime('%Y-%m-%d %H:%M:%S')
        level = record.levelname
        module = (self.swaglogger.get_ctx().get('module', 'unknown') 
                 if self.swaglogger else 'unknown')

        # 处理消息内容
        try:
            if isinstance(record.msg, dict):
                msg = str(record.msg)
            else:
                try:
                    msg = record.getMessage()
                except (ValueError, TypeError):
                    args_list = list(record.args) if record.args else []
                    msg = str([record.msg] + args_list)
        except Exception:
            msg = str(record.msg)

        # 处理异常信息
        error_info = ""
        if record.exc_info:
            try:
                error_info = f" | Error: {self.formatException(record.exc_info).replace(chr(10), ' ')}"
            except Exception:
                error_info = " | Error: <格式化异常信息失败>"

        return f"{time_str} | {level:7s} | {module:15s} | {msg}{error_info}"

# 添加类别名用于向后兼容
SwagLogFileFormatter = SwagFormatter

class SwagErrorFilter(logging.Filter):
    def filter(self, record):
        return record.levelno < logging.ERROR

def _tmpfunc():
    return 0

def _srcfile():
    return os.path.normcase(_tmpfunc.__code__.co_filename)

class SwaglogRotatingFileHandler(BaseRotatingHandler):
    """自定义的日志文件处理器，支持按大小和时间滚动"""
    
    def __init__(self, base_filename, session_id=None, module_name=None, 
                 interval=DEFAULT_INTERVAL, max_bytes=DEFAULT_MAX_BYTES, 
                 backup_count=DEFAULT_BACKUP_COUNT, encoding=DEFAULT_ENCODING, 
                 error_log=False):
        """初始化处理器"""
        self.base_filename = base_filename
        self.session_id = session_id or get_boot_time()
        self.module_name = module_name
        self.interval = interval
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self.error_log = error_log
        
        # 文件管理
        self.log_files = self.get_existing_logfiles()
        log_indexes = [f.split(".")[-1] for f in self.log_files]
        self.last_file_idx = max([int(i) for i in log_indexes if i.isdigit()] or [-1])
        
        # 状态跟踪
        self.last_rollover = time.monotonic()
        self._current_size = 0
        self._file_created = False
        
        super().__init__(base_filename, mode="a", encoding=encoding, delay=True)

    def shouldRollover(self, record):
        """检查是否需要滚动日志文件"""
        if not self.stream:
            return False

        try:
            msg_size = len(self.format(record).encode(self.encoding or DEFAULT_ENCODING))
            self._current_size += msg_size

            size_exceeded = self.max_bytes > 0 and self._current_size >= self.max_bytes
            time_exceeded = (self.interval > 0 and 
                           self.last_rollover + self.interval <= time.monotonic())

            if time_exceeded and self._current_size < MIN_FILE_SIZE:
                self.last_rollover = time.monotonic()
                return False

            return size_exceeded or time_exceeded
        except (AttributeError, OSError):
            return True

    def emit(self, record):
        """输出日志记录"""
        try:
            if not self._file_created:
                if not self.error_log or (self.error_log and record.levelno >= logging.ERROR):
                    self._file_created = True
                else:
                    return

            if self.shouldRollover(record):
                self.doRollover()

            if not self.stream:
                try:
                    self.stream = self._open()
                    if not self.stream:
                        raise ValueError("无法创建日志流")
                except Exception as e:
                    self.handleError(record)
                    print(f"打开日志流失败: {str(e)}")
                    return

            try:
                msg = self.format(record)
                self.stream.write(msg + self.terminator)
                self.stream.flush()
            except Exception as e:
                self.handleError(record)
                print(f"日志写入失败: {str(e)}")
                try:
                    self.stream.close()
                except:
                    pass
                self.stream = None

        except Exception as e:
            self.handleError(record)
            print(f"日志处理失败: {str(e)}")

    def _open(self):
        """打开新的日志文件"""
        try:
            self.last_rollover = time.monotonic()
            self.last_file_idx += 1
            date_str = time.strftime("%Y%m%d_%H%M%S")

            base_name = self.module_name or os.path.basename(self.base_filename)
            if self.error_log:
                base_name = f"{base_name}_error"

            max_attempts = 1000
            attempt = 0
            next_filename = None
            
            while attempt < max_attempts:
                next_filename = os.path.join(
                    os.path.dirname(self.base_filename),
                    f"{base_name}.{date_str}.{self.session_id}.{self.last_file_idx:03d}.log"
                )
                
                if not os.path.exists(next_filename):
                    break
                    
                self.last_file_idx += 1
                attempt += 1
            
            if attempt >= max_attempts:
                raise RuntimeError(f"无法创建唯一的日志文件名: {next_filename}")

            self._current_size = 0
            os.makedirs(os.path.dirname(next_filename), exist_ok=True)
            
            try:
                stream = open(next_filename, self.mode, encoding=self.encoding)
                self.log_files.insert(0, next_filename)
                return stream
            except OSError as e:
                raise RuntimeError(f"无法打开日志文件 {next_filename}: {str(e)}")
                
        except Exception as e:
            print(f"创建日志文件失败: {str(e)}")
            return io.StringIO()

    def get_existing_logfiles(self):
        """获取已存在的日志文件列表"""
        log_files = []
        base_dir = os.path.dirname(self.base_filename)
        if os.path.exists(base_dir):
            for fn in os.listdir(base_dir):
                fp = os.path.join(base_dir, fn)
                if fp.startswith(self.base_filename) and os.path.isfile(fp):
                    log_files.append(fp)
        return sorted(log_files)

    def doRollover(self):
        """执行日志文件滚动"""
        if self.stream:
            try:
                current_size = self.stream.tell()
                if current_size == 0 or current_size < MIN_FILE_SIZE:
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
                        os.remove(to_delete)
                except Exception:
                    continue

def get_custom_file_handler(log_dir, module_name=None):
    """获取自定义文件处理器"""
    try:
        if module_name is None:
            frame = inspect.currentframe()
            caller_frame = frame.f_back
            if caller_frame:
                module = inspect.getmodule(caller_frame)
                if module:
                    module_name = module.__name__.split('.')[-1]
                else:
                    caller_frame = caller_frame.f_back
                    if caller_frame:
                        module = inspect.getmodule(caller_frame)
                        if module:
                            module_name = module.__name__.split('.')[-1]

        module_name = module_name or "unknown"
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        base_filename = os.path.join(log_dir, module_name)

        handler = SwaglogRotatingFileHandler(
            base_filename,
            session_id=get_boot_time(),
            module_name=module_name,
            max_bytes=DEFAULT_MAX_BYTES,
            interval=DEFAULT_INTERVAL,
            backup_count=DEFAULT_BACKUP_COUNT
        )

        return handler, None

    except Exception as e:
        print(f"创建自定义文件处理器失败: {str(e)}")
        return None

class SwagLogger(logging.Logger):
    """自定义日志记录器"""
    def __init__(self):
        super().__init__("swaglog")
        self.global_ctx = {}
        self.log_local = local()
        self.log_local.ctx = {}
        self._custom_handlers = {}
        self._error_handlers = {}
        self._log_dirs = set()  # 用于跟踪已添加的日志目录

    def add_custom_handler(self, log_dir, module_name=None):
        """添加自定义处理器"""
        handler_key = f"{log_dir}:{module_name or self.get_ctx().get('module', 'unknown')}"

        if handler_key not in self._custom_handlers:
            handlers = get_custom_file_handler(log_dir, module_name)
            if handlers:
                handler, error_handler = handlers

                handler.setFormatter(SwagFormatter(self))
                handler.addFilter(SwagErrorFilter())
                handler.setLevel(logging.DEBUG)
                self.addHandler(handler)
                self._custom_handlers[handler_key] = handler

                if error_handler:
                    error_handler.setFormatter(SwagFormatter(self))
                    error_handler.setLevel(logging.ERROR)
                    self.addHandler(error_handler)
                    self._error_handlers[handler_key] = error_handler
                return True
        return False

    def remove_custom_handler(self, log_dir, module_name=None):
        """移除自定义处理器"""
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
        """重写日志记录方法"""
        # 处理带 log_dir 参数的旧格式调用
        log_dir = kwargs.pop('log_dir', None)
        module_name = kwargs.pop('module_name', None)
        
        old_ctx = None
        if module_name:
            old_ctx = self.get_ctx().copy()
            self.bind(module=module_name)

        try:
            if not msg and not args and not exc_info:
                return

            if log_dir:
                self.add_custom_handler(log_dir, module_name)

            if kwargs:
                self.bind(**kwargs)

            super()._log(level, msg, args, exc_info, extra, stack_info, stacklevel)

        finally:
            if module_name and old_ctx:
                self.log_local.ctx.clear()
                self.bind(**old_ctx)

    def local_ctx(self):
        """获取本地上下文"""
        try:
            return self.log_local.ctx
        except AttributeError:
            self.log_local.ctx = {}
            return self.log_local.ctx

    def get_ctx(self):
        """获取完整上下文"""
        return dict(self.local_ctx(), **self.global_ctx)

    @contextmanager
    def ctx(self, **kwargs):
        """上下文管理器"""
        old_ctx = self.local_ctx()
        self.log_local.ctx = copy.copy(old_ctx) or {}
        self.log_local.ctx.update(kwargs)
        try:
            yield
        finally:
            self.log_local.ctx = old_ctx

    def bind(self, **kwargs):
        """绑定本地上下文"""
        self.local_ctx().update(kwargs)

    def bind_global(self, **kwargs):
        """绑定全局上下文"""
        self.global_ctx.update(kwargs)

    def event(self, event, *args, **kwargs):
        """记录事件"""
        evt = NiceOrderedDict()
        evt['event'] = event
        if args:
            evt['args'] = args
        evt.update(kwargs)
        
        formatted_msg = f"Event: {event}"
        if args:
            formatted_msg += f" | Args: {args}"
        for k, v in kwargs.items():
            if k not in ['error', 'debug']:
                formatted_msg += f" | {k}: {v}"
        
        if 'error' in kwargs:
            self.error(formatted_msg)
        elif 'debug' in kwargs:
            self.debug(formatted_msg)
        else:
            self.info(formatted_msg)
        
        return evt

    def timestamp(self, event_name):
      """记录时间戳事件"""
      if LOG_TIMESTAMPS:
          t = time.monotonic()
          tstp = NiceOrderedDict()
          tstp['timestamp'] = NiceOrderedDict()
          tstp['timestamp']["event"] = event_name
          tstp['timestamp']["time"] = t*1e9
          self.debug(tstp)

    def findCaller(self, stack_info=False, stacklevel=1):
        """查找调用者信息"""
        try:
            # 获取初始帧，跳过日志系统内部帧
            f = sys._getframe(3)
            if f is None:
                return "(unknown file)", 0, "(unknown function)", None
                
            # 根据stacklevel调整帧
            orig_f = f
            while f and stacklevel > 1:
                f = f.f_back
                stacklevel -= 1
            if not f:
                f = orig_f

            # 查找实际的调用者
            while hasattr(f, "f_code"):
                try:
                    co = f.f_code
                    if not co:
                        f = f.f_back
                        continue
                        
                    filename = os.path.normcase(co.co_filename)
                    if filename == _srcfile():
                        f = f.f_back
                        continue
                        
                    # 获取堆栈信息
                    sinfo = None
                    if stack_info:
                        sio = io.StringIO()
                        sio.write('Stack (most recent call last):\n')
                        traceback.print_stack(f, file=sio)
                        sinfo = sio.getvalue().rstrip('\n')
                        sio.close()
                        
                    return co.co_filename, f.f_lineno, co.co_name, sinfo
                except AttributeError:
                    f = f.f_back
                    continue
                    
            return "(unknown file)", 0, "(unknown function)", None
            
        except Exception:
            return "(unknown file)", 0, "(unknown function)", None

def init_logger():
    """初始化日志记录器"""
    log = SwagLogger()

    # 设置标准输出处理器
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging.INFO)
    stdout_handler.addFilter(SwagErrorFilter())
    log.addHandler(stdout_handler)

    # 设置标准错误处理器
    stderr_handler = logging.StreamHandler(sys.stderr)
    stderr_handler.setLevel(logging.ERROR)
    log.addHandler(stderr_handler)

    # 设置格式化器
    formatter = SwagFormatter(log)
    stdout_handler.setFormatter(formatter)
    stderr_handler.setFormatter(formatter)

    return log

if __name__ == "__main__":
    log = init_logger()

    # 基本日志测试
    log.info("测试日志 %s", "参数")
    log.info({'数据': 1})
    log.warning("警告信息")
    log.error("错误信息")
    log.critical("严重错误")
    log.event("测试事件", x="y")

    # 上下文测试
    with log.ctx():
        log.bind(user="测试用户")
        log.info("带上下文的日志")
        log.warning("带上下文的警告")
        log.error("带上下文的错误")
        log.critical("带上下文的严重错误")