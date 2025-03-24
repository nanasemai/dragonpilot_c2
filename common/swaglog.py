"""
日志系统核心模块，提供以下功能：

模块职责：
1. 实现跨进程的日志收集和转发机制
2. 提供结构化日志记录能力
3. 统一管理控制台输出和日志服务端转发

设计目标：
- 解耦日志产生与写入操作
- 支持集中式日志管理（通过logmessaged服务）
- 提供模块化上下文追踪能力

使用示例：
1. 基础日志记录：
   cloudlog.info("系统启动")

2. 带模块标识的日志：
   cloudlog.warning("传感器异常", module="sensor")

3. 结构化日志：
   cloudlog.error({
       "event": "network_error",
       "retry_count": 3,
       "endpoint": "api/v1/connect"
   })

主要组件及其职责：
1. UnixDomainSocketHandler - 基于ZMQ的日志转发处理器
   ▹ 实现IPC通信管理（连接/重连机制）
   ▹ 处理多进程资源隔离（PID检测）
   ▹ 非阻塞式网络传输

2. SwagLogManager - 日志系统管理中枢
   ▹ 处理器配置（控制台/套接字）
   ▹ 动态日志方法包装（debug/info/warning/error）
   ▹ 调用栈元数据自动注入

3. SwagFormatter - 结构化日志格式化（在logging_extra.py实现）
   ▹ 上下文信息整合
   ▹ 自定义目录标记提取
   ▹ 多进程安全格式化

全局实例说明：
- log_manager: 单例日志管理器（线程安全初始化）
- cloudlog/log: 统一日志接口（支持上下文绑定）

数据流向：
应用模块 → SwagLogger → 控制台输出
                  ↳→ UnixDomainSocketHandler → ZMQ IPC → logmessaged
                                                      ↳→ 文件系统/网络存储

扩展性说明：
1. 新增日志处理器：通过_setup_handlers()添加新Handler
2. 定制日志格式：修改SwagFormatter实现
3. 调整日志级别：通过DP参数或环境变量实时生效
"""
import time  # 新增导入
import json
import logging
import os
import traceback
import warnings
import zmq
from pathlib import Path
from openpilot.common.logging_extra import (
    SwagLogger, SwagFormatter, json_robust_dumps
)
from openpilot.common.params import Params

MEDIA_PATH = "/data/media/0/c2_logs/swaglog"
DEFAULT_PATH = "/data/log/"
SWAGLOG_DIR = MEDIA_PATH if os.path.exists("/data/media/0") else DEFAULT_PATH

def formatted_print(level, module, message):
    """使用统一格式打印消息"""
    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    print(f"{current_time} | {level} | {module} | {message}")


class UnixDomainSocketHandler(logging.Handler):
    """Unix域套接字处理器，用于日志转发"""
    def __init__(self, formatter):
        super().__init__()
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
        # 检查进程ID是否变化，如果变化则重新连接
        if os.getpid() != self.pid:
            warnings.filterwarnings("ignore", category=ResourceWarning, message="unclosed.*<zmq.*>")
            try:
                self.connect()
            except zmq.error.ZMQError:
                return

        try:
            # 获取模块名，优先从record.module获取
            module = getattr(record, 'module', 'unknown')

            # 如果消息是字典且包含module字段，使用该字段
            if isinstance(record.msg, dict) and 'module' in record.msg:
                module = record.msg['module']

            # 简化消息结构
            msg_content = record.msg
            if isinstance(msg_content, dict):
                msg_content = msg_content.get('msg', str(msg_content))

            # 去除 ANSI 颜色代码
            import re
            msg_content = re.sub(r'\x1b\[[0-9;]*m', '', str(msg_content))

            raw_data = {
                'level': record.levelno,
                'msg': msg_content,  # 使用去除颜色代码后的消息
                'module': module,
                'timestamp': time.time()
            }

            # 编码并发送消息
            payload = b'\x01' + json_robust_dumps(raw_data).encode('utf-8')
            self.sock.send(payload, zmq.NOBLOCK)
        except zmq.error.Again:
            pass  # 队列满时正常忽略
        except Exception as e:
            # 简化异常处理，只记录错误类型和消息
            print(f"LogEmitError({type(e).__name__}): {str(e)}")

    def format(self, record):
        try:
            if isinstance(record.msg, dict):
                msg = record.msg.copy()
            else:
                raw_msg = str(record.msg)
                try:
                    msg = json.loads(raw_msg)
                except json.JSONDecodeError:
                    msg = {'msg': raw_msg}

            if self.swaglogger and isinstance(msg, dict):
                ctx = self.swaglogger.get_ctx() or {}
                msg = {**ctx, **msg}

            # 提取实际消息内容
            if isinstance(msg, dict):
                msg_content = msg.get('msg', str(msg))
            else:
                msg_content = str(msg)

            # 设置记录的消息为提取的内容
            record.msg = msg_content

            return super().format(record)
        except Exception as e:
            return f"FormatterError: {str(e)}"

class SwaglogRotatingFileHandler(logging.handlers.BaseRotatingHandler):
    """滚动日志文件处理器，支持大小和时间触发滚动"""
    def __init__(self, base_filename, interval=60, max_bytes=1024*256, backup_count=2500, encoding=None, startup_time=None):
        super().__init__(base_filename, mode="a", encoding=encoding, delay=True)
        self.base_filename = base_filename
        self.interval = interval  # 秒
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        # 保存启动时间，如果未提供则使用当前时间
        self.startup_time = startup_time or time.strftime("%Y%m%d_%H%M%S")
        self.log_files = self.get_existing_logfiles()
        log_indexes = [f.split(".")[-1] for f in self.log_files]
        self.last_file_idx = max([int(i) for i in log_indexes if i.isdigit()] or [-1])
        self.last_rollover = None
        self.doRollover()

    def _open(self):
        self.last_rollover = time.monotonic()
        self.last_file_idx += 1
        # 在文件名中添加启动时间
        next_filename = f"{self.base_filename}.{self.startup_time}.{self.last_file_idx:010}"
        stream = open(next_filename, self.mode, encoding=self.encoding)
        self.log_files.insert(0, next_filename)
        return stream

    def get_existing_logfiles(self):
        log_files = list()
        base_dir = os.path.dirname(self.base_filename)
        # 修改文件匹配逻辑，考虑启动时间
        for fn in os.listdir(base_dir):
            fp = os.path.join(base_dir, fn)
            if fp.startswith(self.base_filename) and os.path.isfile(fp):
                log_files.append(fp)
        return sorted(log_files)

    def shouldRollover(self, record):
        size_exceeded = self.max_bytes > 0 and self.stream.tell() >= self.max_bytes
        time_exceeded = self.interval > 0 and self.last_rollover + self.interval <= time.monotonic()
        return size_exceeded or time_exceeded

    def doRollover(self):
        if self.stream:
            self.stream.close()
        self.stream = self._open()

        if self.backup_count > 0:
            while len(self.log_files) > self.backup_count:
                to_delete = self.log_files.pop()
                if os.path.exists(to_delete):  # 安全检查
                    os.remove(to_delete)

class AnsiColorStripFormatter(logging.Formatter):
    """去除ANSI颜色代码的格式化器"""
    def __init__(self, orig_formatter):
        super().__init__()
        self.orig_formatter = orig_formatter

    def format(self, record):
        import re
        # 先使用原始格式化器格式化
        formatted = self.orig_formatter.format(record)
        # 去除所有ANSI颜色代码
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', formatted)

def get_file_handler():
    """获取文件日志处理器"""
    Path(SWAGLOG_DIR).mkdir(parents=True, exist_ok=True)
    base_filename = os.path.join(SWAGLOG_DIR, "swaglog")
    # 获取当前启动时间
    startup_time = time.strftime("%Y%m%d_%H%M%S")
    handler = SwaglogRotatingFileHandler(base_filename, startup_time=startup_time)
    return handler

def add_file_handler(log):
    """
    添加文件日志处理器到swaglog
    当logmessaged不运行时可用于存储日志
    """
    handler = get_file_handler()
    # 使用AnsiColorStripFormatter包装原始格式化器，去除颜色代码
    orig_formatter = SwagFormatter(log)
    handler.setFormatter(AnsiColorStripFormatter(orig_formatter))

    # 根据dp_log_level设置文件日志级别
    params = Params()
    dp_log_level = params.get("dp_log_level", encoding='utf8')
    if dp_log_level is not None:
        level_map = {
            "0": logging.WARNING,
            "1": logging.INFO,
            "2": logging.DEBUG
        }
        handler.setLevel(level_map.get(dp_log_level, logging.INFO))
    else:
        handler.setLevel(logging.INFO)

    log.addHandler(handler)

class SwagLogManager:
    """日志管理器，处理日志配置和格式化"""
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SwagLogManager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        # 确保初始化代码只执行一次
        if SwagLogManager._initialized:
            return
        SwagLogManager._initialized = True

        self.logger = SwagLogger()
        self.logger.setLevel(logging.DEBUG)

        # 清除所有现有处理器
        self.logger.handlers.clear()

        # 设置日志处理器
        self._setup_handlers()
        self._wrap_log_methods()

    def _get_console_log_level(self):
        """获取控制台日志级别"""
        params = Params()
        dp_log_level = params.get("dp_log_level", encoding='utf8')

        if dp_log_level is not None:
            level_map = {"0": "warning", "1": "info", "2": "debug"}
            print_level = level_map.get(dp_log_level, "warning")
        else:
            print_level = os.environ.get('LOGPRINT', 'warning')

        return {
            'debug': logging.DEBUG,
            'info': logging.INFO,
            'warning': logging.WARNING
        }.get(print_level, logging.WARNING)

    def _setup_handlers(self):
        """设置日志处理器"""
        # 清除所有现有处理器
        self.logger.handlers.clear()

        # 使用控制台和文件处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(self._get_console_log_level())
        console_handler.setFormatter(SwagFormatter(swaglogger=self.logger))
        self.logger.addHandler(console_handler)


        # 根据logmessaged可用性决定使用哪个处理器
        if self._is_logmessaged_available():
          # 只使用socket_handler
          socket_handler = UnixDomainSocketHandler(SwagFormatter(swaglogger=self.logger))
          socket_handler.setLevel(logging.DEBUG)
          self.logger.addHandler(socket_handler)
          formatted_print("INFO", "swaglog", "logmessaged可用，使用网络日志转发")
        else:
          add_file_handler(self.logger)
          formatted_print("INFO", "swaglog", "logmessaged不可用，已添加本地文件日志备份")

    def _is_logmessaged_available(self):
        """检查logmessaged服务是否可用"""
        try:
            # 尝试连接logmessaged服务
            test_ctx = zmq.Context()
            test_sock = test_ctx.socket(zmq.PUSH)
            test_sock.setsockopt(zmq.LINGER, 0)  # 不等待，立即返回
            test_sock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms超时
            test_sock.connect("ipc:///tmp/logmessage")

            # 尝试发送一个测试消息
            test_data = {
                'level': logging.INFO,
                'msg': 'Testing logmessaged connection',  # 直接使用字符串作为消息内容
                'module': 'swaglog',
                'timestamp': time.time()
            }
            payload = b'\x01' + json_robust_dumps(test_data).encode('utf-8')
            test_sock.send(payload, zmq.NOBLOCK)

            # 清理资源
            test_sock.close()
            test_ctx.term()
            return True
        except zmq.error.Again:
            # 队列满但服务可用
            return True
        except Exception:
            # 连接失败，服务不可用
            return False


    def _get_caller_info(self):
        """获取调用者信息 - 简化版"""
        try:
            for frame in traceback.extract_stack()[-5:-1]:  # 限制搜索范围
                if not frame.filename.endswith('swaglog.py'):
                    return {'file': frame.filename, 'line': frame.lineno}
        except:
            pass
        return None

    def _create_log_record(self, level_name, formatted_msg, current_module):
        """创建日志记录 - 简化版"""
        return logging.LogRecord(
            name=current_module or 'unknown',
            level=logging.getLevelName(level_name),
            pathname='',  # 简化，不需要完整路径
            lineno=0,
            msg=formatted_msg,
            args=(),
            exc_info=None,
            func=None
        )

    def _wrap_log_method(self, original_method, level_name):
        def wrapped_method(msg, *args, **kwargs):
            if not msg:
                return None

            try:
                # 提取模块名
                module = kwargs.pop('module', None)
                if not module:
                    for frame in traceback.extract_stack()[-5:-1]:
                        if not frame.filename.endswith('swaglog.py'):
                            module = Path(frame.filename).stem
                            break

                # 确保模块名不为空
                if not module:
                    module = 'unknown'

                # 简化消息处理
                if isinstance(msg, dict):
                    msg_payload = msg.copy()  # 创建副本避免修改原始数据
                    # 确保消息中包含模块名
                    if 'module' not in msg_payload:
                        msg_payload['module'] = module
                else:
                    msg_str = str(msg)
                    if args:
                        try:
                            msg_str = msg_str % args
                        except:
                            pass
                    msg_payload = {'msg': msg_str, 'module': module}

                # 检查是否为启动日志，如果是则强制打印到控制台
                is_startup_log = False
                if isinstance(msg_payload.get('msg'), str):
                    msg_content = msg_payload.get('msg', '')
                    is_startup_log = '启动' in msg_content or 'start' in msg_content.lower()

                # 创建并处理记录
                record = self._create_log_record(level_name, msg_payload, module)
                # 确保记录对象有module属性
                record.module = module

                # 确保raw_msg属性存在，供SwagFormatter使用
                record.raw_msg = msg_payload

                # 对于启动日志，如果当前日志级别不足以显示，则使用格式化器格式化后打印
                if is_startup_log and level_name in ('INFO', 'DEBUG') and self._get_console_log_level() > logging.INFO:
                    # 使用与SwagFormatter一致的格式化方式
                    formatter = SwagFormatter(swaglogger=self.logger)
                    formatted_msg = formatter.format(record)
                    print(formatted_msg)
                else:
                    self.logger.handle(record)

            except Exception as e:
                formatted_print("ERROR", "swaglog", f"LogError: {e}")
            return None
        return wrapped_method

    def _wrap_log_methods(self):
        """包装所有日志方法"""
        original_methods = {
            'debug': self.logger.debug,
            'info': self.logger.info,
            'warning': self.logger.warning,
            'error': self.logger.error
        }

        for level, method in original_methods.items():
            setattr(self.logger, level, self._wrap_log_method(method, level.upper()))

# 创建全局日志实例
log_manager = SwagLogManager()
cloudlog = log = log_manager.logger
