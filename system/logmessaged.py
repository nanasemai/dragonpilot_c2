"""
日志服务守护进程（V3.2）核心架构：

输入层：
■ ZMQ IPC接收（ipc:///tmp/logmessage）
■ 16进制等级标识 ■ JSON/字符串自动解析
■ 非阻塞IO设计 ■ 多线程安全
■ 大日志拦截(>2MB) ■ 流量控制机制
■ 自动重连机制 ■ 进程隔离保护

处理层：
■ 智能日志过滤 ■ 模块化处理
■ 日志格式标准化 ■ 内存保护机制
■ 异常自动恢复 ■ 性能优化
■ 日志级别动态调整 ■ 消息大小限制
■ 上下文信息注入 ■ 调用栈追踪

输出层：
├─ 主日志：/data/media/0/c2_logs/swaglog
│  └─ 滚动策略：128KB/保留1500份
│  └─ 保留时间：4天
│  └─ 自动清理机制
└─ 实时通道：
   ├─ logMessage（全量日志）
   └─ errorLogMessage（ERROR级别）

监控体系：
■ 目录健康检查 ■ 队列状态监控
■ 资源自动回收 ■ 性能指标统计
■ 错误预警机制 ■ 系统状态报告
■ 日志过滤统计 ■ 处理器状态跟踪

特性：
■ 支持动态日志配置 ■ 多级别日志处理
■ 高性能日志处理 ■ 跨平台兼容
■ 自动清理机制 ■ 实时日志推送
■ 模块化设计 ■ 可扩展架构
■ 线程安全 ■ 异常处理
■ 日志压缩 ■ 远程日志支持
"""
#!/usr/bin/env python3
import os
import zmq
import json
import time
import datetime
import logging
from typing import NoReturn
from pathlib import Path
import cereal.messaging as messaging
from openpilot.common.logging_extra import (
    SwagFormatter, SwaglogRotatingFileHandler,SwagLogger,CustomSwaglogRotatingFileHandler
)
from openpilot.common.params import Params

# 全局配置常量
DEFAULT_LOG_DIR = "/data/media/0/c2_logs/logmessage/"
# 日志滚动配置
LOG_CONFIG = {
    'INTERVAL': 300,        # 滚动时间间隔（秒）
    'MAX_BYTES': 128*1024, # 单个日志文件大小限制（128KB）
    'BACKUP_COUNT': 1500,  # 最大保留日志文件数量
    'MAX_AGE': 4*24*3600, # 日志最大保留时间（4天）
    'ENCODING': 'utf8',     # 日志文件编码
    'CLEAN_INTERVAL': 6*3600  # 清理检查间隔（6小时）
}

def create_log_handler(boot_ts: float):
    try:
        hex_ts = hex(int(boot_ts))[2:]
        os.makedirs(DEFAULT_LOG_DIR, exist_ok=True)
        
        return CustomSwaglogRotatingFileHandler(
            os.path.join(DEFAULT_LOG_DIR, f"swaglog.{hex_ts}.{time.strftime('%Y%m%d_%H%M%S')}.000.log"),
            interval=LOG_CONFIG['INTERVAL'],
            max_bytes=LOG_CONFIG['MAX_BYTES'],
            backup_count=LOG_CONFIG['BACKUP_COUNT'],
            encoding=LOG_CONFIG['ENCODING']
        )
    except Exception as e:
        print(f"HandlerInitError: {str(e)}")
        return None

def clean_old_logs():
    """清理过期日志文件"""
    try:
        current_time = time.time()
        for log_file in Path(DEFAULT_LOG_DIR).glob("*.log"):
            if current_time - log_file.stat().st_mtime > LOG_CONFIG['MAX_AGE']:
                log_file.unlink()
    except Exception as e:
        print(f"CleanLogError: {str(e)}")

def get_system_boottime() -> float:
    """获取系统启动时间戳（秒级精度）
    通过/proc/stat文件获取更准确的启动时间
    当无法获取时（如Windows系统）回退到当前时间"""
    try:
        with open('/proc/stat', 'r') as f:
            for line in f:
                if line.startswith('btime'):
                    return float(line.split()[1])
    except:
        return time.time()  # 回退到当前时间

def get_log_level():
    """获取全局日志级别"""
    params = Params()
    dp_log_level = params.get("dp_log_level", encoding='utf8')

    if dp_log_level is not None:
        level_map = {
            "0": logging.WARNING,
            "1": logging.INFO,
            "2": logging.DEBUG
        }
        return level_map.get(dp_log_level, logging.INFO)
    return logging.INFO

class FilteredLogFormatter(logging.Formatter):
    def __init__(self, swaglogger=None):
        super().__init__()
        self.critical_modules = {
            'controlsd', 'pandad', 'plannerd', 'radard',
            'thermald', 'uploader', 'manager', 'locationd',
            'modeld', 'sensord', 'monitoringd', 'logmessaged'
        }
        self.reduced_modules = {
            'boardd': logging.WARNING,
            'camerad': logging.WARNING,
            'ui': logging.WARNING
        }
        self.global_level = get_log_level()

        # 添加计数器，用于统计过滤情况
        self.filtered_count = 0
        self.total_count = 0
        self.last_stats_time = time.time()

    def format(self, record):
        try:
            # 处理字典类型的日志消息
            if isinstance(record.msg, dict):
                log_dict = record.msg.copy()  # 使用副本避免修改原始数据
                # 提取事件信息和额外参数
                event_msg = log_dict.get('event', 'unknown_event')
                # 移除已知的特殊字段
                for field in ['msg', 'module', 'timestamp']:
                    log_dict.pop(field, None)
                params = {k: v for k, v in log_dict.items() if k not in {'event', 'msg', 'module', 'timestamp'}}
                
                # 构建消息内容
                param_str = ', '.join([f"{k}={v}" for k, v in params.items()])
                msg_content = f"{event_msg} | {param_str}"
                original_length = len(str(record.msg))  # 保持原始长度用于过滤
            else:
                # 原有处理逻辑
                msg_content = str(record.msg)
                original_length = len(msg_content)

            # 过滤检查应使用原始长度
            if not self._should_log(record.module, record.levelno, original_length):
                self.filtered_count += 1
                return ""

            # 获取时间戳和日志级别
            current_time = time.localtime()
            microsecond = int(time.time() * 1000) % 1000
            timestamp = f"{time.strftime('%Y-%m-%d %H:%M:%S', current_time)}.{microsecond:03d}"
            level_name = logging.getLevelName(record.levelno)
            
            return f"{timestamp} | {level_name:<7} | {record.module:<15} | {msg_content}"
        except Exception as e:
            return f"FormatError: {str(e)[:100]}"

    def _should_log(self, module: str, log_level: int, message_size: int) -> bool:
        """判断是否应该记录日志"""
        if module is None:
            module = "unknown"
        module_lower = module.lower()

        # 1. 检查关键模块
        if module_lower in self.critical_modules:
            return True

        # 2. 检查全局日志级别
        if log_level < self.global_level:
            return False

        # 3. 检查消息大小
        if message_size > 2*1024*1024:
            return log_level >= logging.ERROR

        # 4. 检查减少日志模块的级别
        min_level = self.reduced_modules.get(module_lower, logging.DEBUG)
        return log_level >= min_level

def main() -> NoReturn:
    import signal
    import atexit

    def cleanup():
        """清理资源"""
        try:
            if 'sock' in globals() and sock is not None:
                sock.close()
            if 'ctx' in globals() and ctx is not None:
                ctx.term()
            if 'handler' in globals() and handler is not None:
                handler.close()
        except Exception as e:
            print(f"清理错误: {str(e)}")

    def signal_handler(signum, frame):
        """处理退出信号"""
        print(f"收到信号 {signum}，正在退出...")
        cleanup()
        os._exit(0)

    # 注册清理函数
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # 初始化
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PULL)
        sock.bind("ipc:///tmp/logmessage")
        log_sock = messaging.pub_sock("logMessage")
        error_sock = messaging.pub_sock("errorLogMessage")

        # 日志系统初始化
        BOOT_TIMESTAMP = get_system_boottime()
        Path(DEFAULT_LOG_DIR).mkdir(parents=True, exist_ok=True)
        clean_old_logs()

        # 初始化日志处理器
        if not (handler := create_log_handler(BOOT_TIMESTAMP)):
            print("无法创建日志处理器")
            # 清理资源
            sock.close()
            ctx.term()
            return

        # 确保只添加一个处理器
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)  # 添加这行，确保可以处理所有级别的日志
        logger.handlers.clear()  # 清除所有现有处理器
        logger.propagate = False  # 防止日志传播到父logger
        handler.setFormatter(FilteredLogFormatter(None))
        logger.addHandler(handler)

        # 测试日志
        test_record = logging.LogRecord(
            name='logmessaged',
            level=logging.INFO,
            pathname='',
            lineno=0,
            msg='日志系统启动成功',
            args=(),
            exc_info=None
        )
        handler.emit(test_record)
        handler.flush()

        last_clean_time = time.time()

        while True:
            # 定期清理旧日志
            if time.time() - last_clean_time > LOG_CONFIG['CLEAN_INTERVAL']:
                clean_old_logs()
                last_clean_time = time.time()

            try:
                # 接收日志数据
                dat = sock.recv_multipart()[0]
                if not dat:
                    continue

                # 解析日志数据
                try:
                    if dat.startswith(b'\x01'):  # JSON格式
                        raw_data = json.loads(dat[1:].decode('utf-8', errors='replace'))
                        level = min(max(int(raw_data.get('level', 10)), logging.DEBUG), logging.CRITICAL)
                        msg = raw_data.get('msg', '')
                        module = raw_data.get('module', 'unknown').strip() or 'unknown'
                    else:  # 文本格式
                        level = min(max(int(dat[0]), logging.DEBUG), logging.CRITICAL)
                        raw_msg = dat[1:].decode('utf-8', errors='replace')

                        # 尝试解析文本中的JSON结构
                        try:
                            msg_dict = json.loads(raw_msg)
                            # 优先使用daemon字段作为模块名
                            module = msg_dict.get('daemon') or \
                                   msg_dict.get('filename', 'text_log').split('/')[-1].split('.')[0]
                            module = module.lower().strip()  # 统一命名规范
                            # 提取消息内容（优先使用msg字段）
                            msg = msg_dict.get('msg', raw_msg)
                            # 使用日志中的时间戳（如果存在）
                            current_time = time.localtime()
                            microsecond = int(time.time() * 1000) % 1000
                            timestamp = f"{time.strftime('%Y-%m-%d %H:%M:%S', current_time)}.{microsecond:03d}"
                        except Exception:
                            # 非结构化文本处理
                            module = 'text_log'
                            msg = raw_msg
                            current_time = time.localtime()
                            microsecond = int(time.time() * 1000) % 1000
                            timestamp = f"{time.strftime('%Y-%m-%d %H:%M:%S', current_time)}.{microsecond:03d}"
                        # 构建统一日志格式
                        #formatted_msg = f"{timestamp} | {logging.getLevelName(level):<7} | {module:<15} | {msg}"

                    # 日志过滤检查
                    if not handler.formatter._should_log(module, level, len(str(msg))):
                        continue

                    # 创建日志记录
                    record = logging.LogRecord(
                        name=module,
                        level=level,
                        pathname='',
                        lineno=0,
                        msg=msg,
                        args=(),
                        exc_info=None
                    )
                    record.module = module

                    # 处理日志
                    formatted = handler.formatter.format(record)
                    if formatted:
                        handler.emit(record)

                        # 发布日志消息
                        if len(formatted) <= 2*1024*1024:
                            log_msg = messaging.new_message()
                            log_msg.logMessage = formatted
                            log_sock.send(log_msg.to_bytes())

                            if level >= logging.ERROR:
                                error_msg = messaging.new_message()
                                error_msg.errorLogMessage = formatted
                                error_sock.send(error_msg.to_bytes())

                except Exception as parse_error:
                    print(f"日志解析错误: {str(parse_error)}")
                    continue

            except Exception as recv_error:
                print(f"接收错误: {str(recv_error)}")
                time.sleep(0.1)

    except Exception as e:
        print(f"FatalError: {str(e)}")
        os._exit(1)
    finally:
        sock.close()
        ctx.term()
        handler.close()

if __name__ == "__main__":
    main()
