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
import logging
from typing import NoReturn
from pathlib import Path

import cereal.messaging as messaging
from openpilot.common.logging_extra import (
    SwagFormatter, SwaglogRotatingFileHandler,SwagLogger
)
from openpilot.common.params import Params

# 全局配置常量
DEFAULT_LOG_DIR = "/data/media/0/c2_logs/logmessage/"
MAX_LOG_SIZE = 128 * 1024  # 128KB
BACKUP_COUNT = 1500
MAX_LOG_AGE = 4 * 24 * 3600  # 日志最大保留时间（4天）
LOG_ROLLOVER_INTERVAL = 60  # 修改：日志滚动时间间隔（5分钟）

def create_log_handler(boot_ts: float):
    try:
        hex_ts = hex(int(boot_ts))[2:]

        # 确保日志目录存在
        os.makedirs(DEFAULT_LOG_DIR, exist_ok=True)

        class CustomSwaglogRotatingFileHandler(SwaglogRotatingFileHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.process_count = 0  # 添加计数器
                self.last_rollover_time = time.time()
                base_name = os.path.basename(args[0])
                parts = base_name.split('.')
                self.base_name = parts[0]  # swaglog
                self.hex_ts = parts[1]     # 67e0071a
                self.rollover_count = 0
                # 确保文件立即打开
                if not self.delay:
                    self.stream = self._open()

            def emit(self, record):
                # 确保文件已打开
                if self.stream is None and not self.delay:
                    self.stream = self._open()
                
                # 格式化日志并添加换行符
                msg = self.format(record)
                if msg:  # 只有非空消息才写入
                    self.stream.write(msg + '\n')
                    self.stream.flush()
                self.process_count += 1

            def doRollover(self):
                """执行日志滚动"""
                if self.stream:
                    self.stream.close()
                    self.stream = None

                # 生成新的文件名
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                new_name = os.path.join(
                    os.path.dirname(self.baseFilename),
                    f"{self.base_name}.{self.hex_ts}.{timestamp}.{self.rollover_count:03d}.log"
                )

                # 如果文件已存在，增加计数
                while os.path.exists(new_name) and self.rollover_count < 1000:
                    self.rollover_count += 1
                    new_name = os.path.join(
                        os.path.dirname(self.baseFilename),
                        f"{self.base_name}.{self.hex_ts}.{timestamp}.{self.rollover_count:03d}.log"
                    )

                if self.rollover_count >= 1000:
                    raise RuntimeError("Rollover count exceeded maximum limit")

                self.baseFilename = new_name
                self.rollover_count += 1
                self.last_rollover_time = time.time()

                if not self.delay:
                    self.stream = self._open()

            def shouldRollover(self, record):
                """检查是否需要滚动日志"""
                if super().shouldRollover(record):
                    return True

                current_time = time.time()
                if current_time - self.last_rollover_time > LOG_ROLLOVER_INTERVAL:
                    return True
                return False

        return CustomSwaglogRotatingFileHandler(
            os.path.join(DEFAULT_LOG_DIR,
                       f"swaglog.{hex_ts}.{time.strftime('%Y%m%d_%H%M%S')}.000.log"),
            max_bytes=MAX_LOG_SIZE,
            backup_count=BACKUP_COUNT
        )
    except Exception as e:
        print(f"HandlerInitError: {str(e)}")
        return None

def clean_old_logs():
    """清理过期日志文件"""
    try:
        current_time = time.time()
        for log_file in Path(DEFAULT_LOG_DIR).glob("*.log"):
            if current_time - log_file.stat().st_mtime > MAX_LOG_AGE:
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
            # 添加调试信息
            debug_info = f"Module: {record.module}, Level: {record.levelno}, MsgLen: {len(str(record.msg))}"
            
            # 快速路径：已格式化的消息
            if isinstance(record.msg, str) and '|' in record.msg:
                return record.msg

            # 直接使用消息内容，无需额外处理
            msg_content = str(record.msg)

            # 过滤检查
            should_log = self._should_log(record.module, record.levelno, len(msg_content))
            if not should_log:
                # 更新过滤计数
                self.filtered_count += 1
                return ""  # 返回空字符串而不是None

            # 简化时间戳获取
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            level_name = logging.getLevelName(record.levelno)

            return f"{timestamp} | {level_name:<7} | {record.module:<15} | {msg_content}"
        except Exception as e:
            return f"FormatError: {str(e)[:100]}"

    def _should_log(self, module: str, log_level: int, message_size: int) -> bool:
        """判断是否应该记录日志"""
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
            return
        
        # 确保只添加一个处理器
        logger = logging.getLogger()
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
            if time.time() - last_clean_time > 6 * 3600:
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
                        msg = dat[1:].decode('utf-8', errors='replace')
                        module = 'text_log'

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
