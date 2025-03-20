'''
日志系统使用说明:
1. 基础用法:
   cloudlog.debug("调试信息")
   cloudlog.info("普通信息")
   cloudlog.warning("警告信息")
   cloudlog.error("错误信息")
2. 带参数的格式化:
   cloudlog.info("用户 %s 登录", username)
   cloudlog.warning("温度: %.2f", temp)
3. 指定日志目录:
   cloudlog.info("消息内容", log_dir="/data/media/0/custom_logs")
4. 指定模块名:
   cloudlog.info("消息内容", module_name="CustomModule")
5. 同时指定目录和模块:
   cloudlog.info("消息内容", log_dir="/data/media/0/custom_logs", module_name="CustomModule")
6. 带上下文信息:
   with cloudlog.ctx(user="用户名"):
       cloudlog.info("带上下文的消息")
7. 事件记录:
   cloudlog.event("事件名称", value=123, error=True)
8. 时间戳记录:
   cloudlog.timestamp("事件名称")

9. 初始化模块日志(推荐方式):
   # 设置模块名称
   cloudlog.bind_global(module='模块名')
   # 确保日志目录存在
   Path(日志目录).mkdir(parents=True, exist_ok=True)
   # 添加文件处理器
   add_file_handler(cloudlog, 日志目录)
   # 添加文件处理器 (指定文件名前缀)
   add_file_handler(cloudlog, 日志目录, 文件名前缀)

注意事项:
- 不指定 log_dir 时只输出到控制台和默认日志
- 指定 log_dir 后会同时输出到控制台和指定目录
- 不同的 log_dir 会创建独立的日志文件
- module_name 会影响日志文件名和日志内容的模块标识
'''
#rick:
#this file is identical to system/swaglog.py, put a copy here to reduce code changes when updating from op master branch.
import logging
import os
import warnings
import time
import datetime
from pathlib import Path
import zmq
from openpilot.common.logging_extra import (
    SwagLogger, SwagFormatter, SwagLogFileFormatter,
    SwaglogRotatingFileHandler, SwagErrorFilter,
    get_custom_file_handler, get_boot_time
)
from openpilot.system.hardware import PC
from openpilot.common.params import Params

# 日志目录配置
if PC:
    SWAGLOG_DIR = os.path.join(str(Path.home()), ".comma", "log")
else:
    MEDIA_PATH = "/data/media/0/c2_logs/swaglog"
    DEFAULT_PATH = "/data/log/"
    SWAGLOG_DIR = MEDIA_PATH if os.path.exists("/data/media/0") else DEFAULT_PATH




def get_file_handler():
    """获取基础文件处理器"""
    try:
        Path(SWAGLOG_DIR).mkdir(parents=True, exist_ok=True)
        base_filename = os.path.join(SWAGLOG_DIR, "swaglog")
        handler = SwaglogRotatingFileHandler(
            base_filename,
            max_bytes=512*1024,    # 512KB
            interval=300,          # 5分钟
            backup_count=500       # 备份数量
        )
        return handler
    except Exception as e:
        print(f"Error creating file handler: {str(e)}")
        return None

class UnixDomainSocketHandler(logging.Handler):
    """Unix域套接字日志处理器"""
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
            s = chr(record.levelno) + msg
            self.sock.send(s.encode('utf8'), zmq.NOBLOCK)
        except zmq.error.Again:
            pass

def add_file_handler(log, log_dir=None, module_name=None):
    """添加文件处理器"""
    if module_name is None:
        import inspect
        frame = inspect.currentframe().f_back
        module = inspect.getmodule(frame)
        if module:
            module_name = module.__name__.split('.')[-1]
    try:
        # 移除所有现有的文件处理器
        for handler in log.handlers[:]:
            if isinstance(handler, (SwaglogRotatingFileHandler, logging.FileHandler)):
                log.removeHandler(handler)

        # 使用 get_custom_file_handler 创建处理器
        if log_dir:
            handlers = get_custom_file_handler(log_dir, module_name)
            if handlers:
                handler, error_handler = handlers
                if handler:
                    handler.setFormatter(SwagLogFileFormatter(log))
                    handler.addFilter(SwagErrorFilter())
                    log.addHandler(handler)
                if error_handler:
                    error_handler.setFormatter(SwagLogFileFormatter(log))
                    error_handler.setLevel(logging.ERROR)
                    log.addHandler(error_handler)
        else:
            # 使用默认处理器
            handler = get_file_handler()
            if handler:
                handler.setFormatter(SwagLogFileFormatter(log))
                if module_name and hasattr(handler, 'module_name'):
                    handler.module_name = module_name
                log.addHandler(handler)
            
    except Exception as e:
        print(f"Error creating file handler: {str(e)}")

def wrap_log_method(original_method, level_name):
    """包装日志方法"""
    def wrapped_method(msg, *args, **kwargs):
        log_dir = kwargs.pop('log_dir', None)
        module_name = kwargs.pop('module_name', None)

        if not msg and not args:
            return None

        old_ctx = None
        if module_name:
            old_ctx = cloudlog.get_ctx().copy()
            cloudlog.bind(module=module_name)

        try:
            # 移除 log_dir 相关的处理逻辑，统一使用 add_file_handler
            if log_dir or module_name:
                add_file_handler(cloudlog, log_dir, module_name)

            return original_method(msg, *args, **kwargs)

        finally:
            if module_name and old_ctx:
                cloudlog.log_local.ctx = old_ctx

    return wrapped_method

# 初始化日志记录器
cloudlog = log = SwagLogger()
log.setLevel(logging.DEBUG)

# 保存原始日志方法
original_debug = cloudlog.debug
original_info = cloudlog.info
original_warning = cloudlog.warning
original_error = cloudlog.error

# 替换为包装后的日志方法
cloudlog.debug = wrap_log_method(original_debug, "DEBUG")
cloudlog.info = wrap_log_method(original_info, "INFO")
cloudlog.warning = wrap_log_method(original_warning, "WARNING")
cloudlog.error = wrap_log_method(original_error, "ERROR")

# 配置输出处理器
outhandler = logging.StreamHandler()

# 获取日志级别配置
params = Params()
dp_log_level = params.get("dp_log_level", encoding='utf8')

# 设置日志级别
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

# 应用日志级别
if print_level == 'debug':
    outhandler.setLevel(logging.DEBUG)
elif print_level == 'info':
    outhandler.setLevel(logging.INFO)
elif print_level == 'warning':
    outhandler.setLevel(logging.WARNING)

# 添加处理器
log.addHandler(outhandler)
log.addHandler(UnixDomainSocketHandler(SwagFormatter(log)))
