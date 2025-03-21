import datetime
import os
import sys
import time
import logging
import traceback
import threading
from pathlib import Path
from threading import local
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

class SwagLogger(logging.Logger):
    def __init__(self):
        super().__init__("swaglog")
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

class SwagFormatter(logging.Formatter):
    def __init__(self, swaglogger=None):
        super().__init__()
        self.swaglogger = swaglogger

    def format(self, record):
        try:
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:23]
            level = record.levelname
            
            # 获取模块名
            if hasattr(record, 'name'):
                module = record.name
            else:
                module = self.swaglogger.get_ctx().get('module', 'unknown') if self.swaglogger else 'unknown'
            
            # 格式化消息
            msg = str(record.msg)
            
            # 构建完整日志行
            log_parts = [
                timestamp,
                f"{level:7s}",
                f"{module:15s}",
                msg
            ]
            
            if record.exc_info:
                log_parts.append(self.formatException(record.exc_info))
                
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

        # 解析当前文件名的基本部分
        base_dir = os.path.dirname(self.baseFilename)
        base_name = os.path.basename(self.baseFilename)
        
        # 如果文件名已经包含时间戳，则使用新的时间戳
        date_str = time.strftime("%Y%m%d_%H%M%S")
        
        # 生成新的文件名，格式：原始名称.序号.log
        file_idx = 1
        while True:
            new_name = os.path.join(
                base_dir,
                f"{os.path.splitext(base_name)[0]}.{file_idx:03d}.log"
            )
            if not os.path.exists(new_name):
                break
            file_idx += 1

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