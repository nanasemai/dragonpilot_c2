import logging
import os
import traceback
import warnings
import datetime
import threading
import zmq
from openpilot.common.logging_extra import (
    SwagLogger, SwagFormatter
)
from openpilot.common.params import Params

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
        if os.getpid() != self.pid:
            warnings.filterwarnings("ignore", category=ResourceWarning, message="unclosed.*<zmq.*>")
            self.connect()

        try:
            msg = self.format(record).rstrip('\n')
            self.sock.send((chr(record.levelno) + msg).encode('utf8'), zmq.NOBLOCK)
        except zmq.error.Again:
            pass

class SwagLogManager:
    """日志管理器，处理日志配置和格式化"""
    def __init__(self):
        self.logger = SwagLogger()
        self.logger.setLevel(logging.DEBUG)
        self._setup_handlers()
        self._wrap_log_methods()

    def _setup_handlers(self):
        """设置日志处理器"""
        # 只保留控制台输出和消息转发
        console_handler = logging.StreamHandler()
        console_handler.setLevel(self._get_console_log_level())
        console_handler.setFormatter(SwagFormatter(self.logger))
        self.logger.addHandler(console_handler)

        # 转发到 logmessaged
        socket_handler = UnixDomainSocketHandler(SwagFormatter(self.logger))
        socket_handler.setLevel(logging.DEBUG)
        self.logger.addHandler(socket_handler)



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


    def _get_caller_info(self):
        """获取调用者信息"""
        try:
            for frame in traceback.extract_stack()[:-2]:
                if not frame[0].endswith('swaglog.py'):
                    return {
                        'file': frame[0],
                        'line': frame[1],
                        'func': frame[2]
                    }
        except Exception:
            pass
        return None

    def _wrap_log_method(self, original_method, level_name):
        """包装日志方法"""
        def wrapped_method(msg, *args, **kwargs):
            if not msg and not args:
                return None

            # 处理模块名和上下文
            module_name = kwargs.pop('module_name', None)
            log_dir = kwargs.pop('log_dir', None)
            old_ctx = None

            if module_name:
                old_ctx = self.logger.get_ctx().copy()
                self.logger.bind(module=module_name)

            try:
                # 格式化消息
                formatted_msg = str(msg) if isinstance(msg, dict) else (
                    msg % args if args else str(msg)
                )

                # 获取当前模块
                current_module = module_name or self.logger.get_ctx().get('module', 'unknown')

                # 创建日志记录
                record = logging.LogRecord(
                    name=current_module,
                    level=logging.getLevelName(level_name),
                    pathname='',
                    lineno=0,
                    msg=formatted_msg,
                    args=(),
                    exc_info=None
                )

                # 添加自定义目录信息
                if log_dir:
                    record.custom_log_dir = log_dir  # 直接设置属性而不是使用 setattr

                # 发送到处理器
                self.logger.handle(record)

            finally:
                if module_name and old_ctx:
                    self.logger.log_local.ctx.clear()
                    self.logger.log_local.ctx.update(old_ctx)

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