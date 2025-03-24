""" 
增强日志系统模块，提供以下核心功能：

类构成：
1. SwagLogger - 扩展的标准日志记录器
   - bind()/bind_global()：上下文变量绑定
   - event()：结构化事件记录
   - ctx()：上下文管理器
   - timestamp()：高精度时间戳记录
   - 支持模块名自动注入
   - 提供多级别日志方法（info/error/warning/debug）

2. SwagFormatter - 增强型日志格式化器
   - 支持上下文变量注入
   - 异常信息自动格式化
   - 多数据类型处理（字典/字符串/异常对象）
   - 统一日志格式：时间 | 级别 | 模块 | 消息

3. SwaglogRotatingFileHandler - 增强型滚动日志处理器
   - 自定义日志滚动策略
   - 支持UTF-8编码
   - 自动处理日志文件备份

辅助功能：
- JSON安全序列化（json_robust_dumps）：支持复杂对象序列化
- 线程安全的上下文管理
- 日志文件自动清理
"""
import datetime
import os
import time  # 新增导入
import json
import logging
from threading import local
from collections import OrderedDict
from logging.handlers import RotatingFileHandler
from datetime import timezone  

class SwagLogger(logging.Logger):  
    def __init__(self):
        logging.Logger.__init__(self, "swaglog")
        self.global_ctx = {}
        self.log_local = local()
        self.log_local.ctx = {}

    def get_ctx(self):
        if not hasattr(self.log_local, 'ctx'):
            self.log_local.ctx = {}
        return {**self.log_local.ctx, **self.global_ctx}

    def bind(self, **kwargs):
        if not hasattr(self.log_local, 'ctx'):
            self.log_local.ctx = {}
        self.log_local.ctx.update(kwargs)

    def bind_global(self, **kwargs):
        self.global_ctx.update(kwargs)

    def event(self, name, **kwargs):
        """记录事件日志"""
        try:
            event_data = {"event": name}
            event_data.update(kwargs)

            if 'error' in kwargs:
                self.error(event_data)
            else:
                self.info(event_data)
        except Exception as e:
            self.error(f"Failed to log event {name}: {str(e)}")

    def ctx(self):
        from contextlib import contextmanager
        import copy

        @contextmanager
        def _ctx():
            old_ctx = getattr(self.log_local, 'ctx', {})
            self.log_local.ctx = copy.copy(old_ctx) or {}
            try:
                yield
            finally:
                self.log_local.ctx = old_ctx
        return _ctx()

    def timestamp(self, event_name):
        if "LOG_TIMESTAMPS" in os.environ:
            t = time.monotonic()
            tstp = {"timestamp": {"event": event_name, "time": t*1e9}}
            self.debug(tstp)

    def _log(self, level, msg, args, exc_info=None, extra=None, stack_info=False,
             stacklevel=1, module_name=None):
        if module_name:
            self.bind(module=module_name)

        if isinstance(msg, str) and module_name:
            msg_dict = {
                "msg": msg,
                "module": module_name
            }
            msg = msg_dict
            args = ()

        return super()._log(level, msg, args, exc_info, extra, stack_info, stacklevel)

    def info(self, msg, *args, **kwargs):
        return self._log(logging.INFO, msg, args, **kwargs)

    def error(self, msg, *args, **kwargs):
        return self._log(logging.ERROR, msg, args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        return self._log(logging.WARNING, msg, args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        return self._log(logging.DEBUG, msg, args, **kwargs)

class SwagFormatter(logging.Formatter):  
    def __init__(self, swaglogger=None):
        super().__init__()
        self.swaglogger = swaglogger

    def format(self, record):
        try:
            # 简化消息处理逻辑
            if isinstance(record.msg, dict):
                msg = record.msg
            else:
                msg = {'msg': str(record.msg)}
                
            # 只在必要时合并上下文
            if self.swaglogger:
                ctx = self.swaglogger.get_ctx()
                if ctx and isinstance(msg, dict):
                    msg = {**ctx, **msg}
                    
            # 提取消息内容
            msg_content = msg.get('msg', str(msg))
            record.msg = msg_content
            record.raw_msg = msg
            
            # 简化时间戳格式化
            timestamp = datetime.datetime.fromtimestamp(record.created).strftime("%Y-%m-%d %H:%M:%S")
            level_name = logging.getLevelName(record.levelno)
            module = getattr(record, 'module', 'unknown')
            return f"{timestamp} | {level_name} | {module} | {msg_content}"
        except Exception as e:
            return f"FormatterError: {str(e)}"


class SwaglogRotatingFileHandler(RotatingFileHandler):
    """增强型滚动日志处理器"""
    def __init__(self, filename, max_bytes=0, backup_count=0):
        super().__init__(
            filename,
            mode='a',
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding='utf8',
            delay=False
        )
        
    def doRollover(self):
        """自定义滚动策略"""
        if self.stream:
            self.stream.close()
            self.stream = None
        if self.backupCount > 0:
            for i in range(self.backupCount - 1, 0, -1):
                sfn = self.rotation_filename(f"{self.baseFilename}.{i:03d}")
                dfn = self.rotation_filename(f"{self.baseFilename}.{(i + 1):03d}")
                if os.path.exists(sfn):
                    if os.path.exists(dfn):
                        os.remove(dfn)
                    os.rename(sfn, dfn)
            dfn = self.rotation_filename(f"{self.baseFilename}.001")
            if os.path.exists(dfn):
                os.remove(dfn)
            os.rename(self.baseFilename, dfn)
        self.stream = self._open()



def json_robust_dumps(obj):
    """增强型JSON序列化"""
    def handler(o):  # ✅ 修正内部函数参数名冲突问题
        if isinstance(o, (datetime.datetime, datetime.date)):  # 参数重命名为 o
            return o.isoformat()
        if hasattr(o, '__dict__'):
            return vars(o)
        return repr(o)
    return json.dumps(obj, default=handler, ensure_ascii=False)

class NiceOrderedDict(OrderedDict):
    def __str__(self):
        return json_robust_dumps(self)

if __name__ == "__main__":
    log = SwagLogger()
    console = logging.StreamHandler()
    console.setFormatter(SwagFormatter(log))
    log.addHandler(console)
    log.info("测试日志")
    log.bind(module="test")
    log.info("带模块的测试日志")
    log.error("错误日志测试")