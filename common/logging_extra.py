import datetime
import os
import sys
import time
import json
import logging
import traceback
import threading
import uuid
from pathlib import Path
from threading import local
from collections import OrderedDict
from logging.handlers import BaseRotatingHandler

def get_boot_time():
    """获取系统启动时间作为会话ID"""
    try:
        with open('/proc/uptime', 'r') as f:
            uptime_seconds = float(f.readline().split()[0])
        boot_time = int(time.time() - uptime_seconds)
        return f"{boot_time:x}"
    except Exception:
        return f"{int(time.time()):x}"

def _tmpfunc():
    return 0

def _srcfile():
    return os.path.normcase(_tmpfunc.__code__.co_filename)

class SwagLogger(logging.Logger):
    def __init__(self):
        logging.Logger.__init__(self, "swaglog")
        self.global_ctx = {}
        self.log_local = local()
        self.log_local.ctx = {}
        self._custom_handlers = {}
        self._error_handlers = {}

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
            # 构建事件数据
            event_data = {"event": name}
            event_data.update(kwargs)
            
            # 根据是否包含错误信息决定日志级别
            if 'error' in kwargs:
                self.error(event_data)
            else:
                self.info(event_data)
        except Exception as e:
            self.error(f"Failed to log event {name}: {str(e)}")

    # 添加上下文管理器方法
    def ctx(self):
        """创建一个临时的日志上下文"""
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
        """记录时间戳事件"""
        if "LOG_TIMESTAMPS" in os.environ:
            t = time.monotonic()
            tstp = {"timestamp": {"event": event_name, "time": t*1e9}}
            self.debug(tstp)

class SwagFormatter(logging.Formatter):
    def __init__(self, swaglogger=None):
        super().__init__()
        self.swaglogger = swaglogger

    def format(self, record):
        try:
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:23]
            level = record.levelname
            module = self.swaglogger.get_ctx().get('module', 'unknown') if self.swaglogger else 'unknown'
            
            # 处理不同类型的消息
            if isinstance(record.msg, dict):
                msg = str(record.msg)
            else:
                try:
                    msg = record.getMessage()
                except (ValueError, TypeError):
                    msg = str([record.msg] + record.args)
            
            # 构建完整日志行
            log_parts = [
                timestamp,
                f"{level:7s}",
                f"{module:15s}",
                msg
            ]
            
            # 添加异常信息
            if record.exc_info:
                log_parts.append(self.formatException(record.exc_info))
            
            # 添加上下文信息
            ctx = self.swaglogger.get_ctx() if self.swaglogger else {}
            if ctx and ctx != {'module': module}:
                log_parts.append(f"ctx: {ctx}")
                
            return " | ".join(filter(None, log_parts))
            
        except Exception as e:
            return f"日志格式化失败: {str(e)}"

class SwaglogRotatingFileHandler(BaseRotatingHandler):
    def __init__(self, base_filename, module_name=None, max_bytes=512*1024, 
                 interval=300, backup_count=500):
        super().__init__(base_filename, 'a', encoding='utf-8')
        self.module_name = module_name
        self.max_bytes = max_bytes
        self.interval = interval
        self.backup_count = backup_count
        self.session_id = get_boot_time()
        self._current_size = 0
        self.last_rollover = time.monotonic()

    def shouldRollover(self, record):
        if not self.stream:
            return False
            
        msg_size = len(self.format(record).encode('utf-8'))
        self._current_size += msg_size
        
        time_since_last = time.monotonic() - self.last_rollover
        return (self.max_bytes > 0 and self._current_size >= self.max_bytes) or \
               (time_since_last > max(self.interval, 60) and self._current_size >= 1024)

    def doRollover(self):
        if self.stream:
            self.stream.close()
            self.stream = None

        # 解析当前文件名
        base_dir = os.path.dirname(self.baseFilename)
        base_name = os.path.basename(self.baseFilename)
        name_without_index = '.'.join(base_name.split('.')[:-2])  # 移除序号和.log后缀
        
        # 查找当前目录下的所有相关日志文件
        existing_files = [f for f in os.listdir(base_dir) 
                        if f.startswith(name_without_index) and f.endswith('.log')]
        
        # 找到最大的序号
        max_index = 0
        for f in existing_files:
            try:
                # 提取序号部分 (.000.log, .001.log 等)
                index_str = f.split('.')[-2]
                if index_str.isdigit() and len(index_str) == 3:
                    max_index = max(max_index, int(index_str))
            except (ValueError, IndexError):
                continue
        
        # 生成新的文件名，序号加1
        new_name = os.path.join(
            base_dir,
            f"{name_without_index}.{(max_index + 1):03d}.log"
        )

        self.baseFilename = new_name
        self._current_size = 0
        self.last_rollover = time.monotonic()
        
        if self.stream:
            self.stream = self._open()

class SwagErrorFilter(logging.Filter):
    def filter(self, record):
        return record.levelno < logging.ERROR

def get_custom_file_handler(log_dir, module_name):
    try:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        handler = SwaglogRotatingFileHandler(
            os.path.join(log_dir, module_name),
            module_name=module_name
        )
        return handler, None
    except Exception as e:
        print(f"创建自定义文件处理器失败: {str(e)}")
        return None, None

class SwagLogFileFormatter(SwagFormatter):
    def fix_kv(self, k, v):
        """修复键值对的类型标记"""
        if isinstance(v, (str, bytes)):
            k += "$s"
        elif isinstance(v, float):
            k += "$f"
        elif isinstance(v, bool):
            k += "$b"
        elif isinstance(v, int):
            k += "$i"
        elif isinstance(v, dict):
            nv = {}
            for ik, iv in v.items():
                ik, iv = self.fix_kv(ik, iv)
                nv[ik] = iv
            v = nv
        elif isinstance(v, list):
            k += "$a"
        return k, v

    def format(self, record):
        if isinstance(record, str):
            v = json.loads(record)
        else:
            v = self.format_dict(record)

        mk, mv = self.fix_kv('msg', v['msg'])
        del v['msg']
        v[mk] = mv
        v['id'] = uuid.uuid4().hex

        return json_robust_dumps(v)

    def findCaller(self, stack_info=False, stacklevel=1):
        """定位调用者的位置信息"""
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

def json_handler(obj):
    return repr(obj)

def json_robust_dumps(obj):
    return json.dumps(obj, default=json_handler)

class NiceOrderedDict(OrderedDict):
    def __str__(self):
        return json_robust_dumps(self)

# 用于测试的代码
if __name__ == "__main__":
    log = SwagLogger()
    
    # 设置控制台处理器
    console = logging.StreamHandler()
    console.setFormatter(SwagFormatter(log))
    log.addHandler(console)
    
    # 测试日志输出
    log.info("测试日志")
    log.bind(module="test")
    log.info("带模块的测试日志")
    log.error("错误日志测试")