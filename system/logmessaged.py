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
    SwagFormatter, SwaglogRotatingFileHandler
)
from openpilot.common.params import Params

# 日志目录配置
MEDIA_PATH = "/data/media/0/c2_logs/swaglog"
DEFAULT_PATH = "/data/log/"
SWAGLOG_DIR = MEDIA_PATH if os.path.exists("/data/media/0") else DEFAULT_PATH

class FilteredLogFormatter(SwagFormatter):
  def __init__(self, swaglogger=None):
    super().__init__(swaglogger)
    self.critical_modules = {
      'controlsd', 'pandad', 'plannerd', 'radard',
      'thermald', 'uploader', 'manager', 'locationd',
      'modeld'
    }
    self.reduced_modules = {
      'boardd': logging.WARNING,
      'logmessaged': logging.WARNING,
      'camerad': logging.WARNING,
      'ui': logging.WARNING
    }
    self.current_record = None

  def _should_log(self, module: str, log_level: int, message_size: int = 0) -> bool:
    if 'gpu' in module.lower() or module == 'modeld':
      return True

    if message_size > 2*1024*1024 and log_level < logging.ERROR:
      return False

    if log_level >= logging.ERROR:
      return True

    if module in self.critical_modules:
      return True

    if module in self.reduced_modules:
      return log_level >= self.reduced_modules[module]

    return True

  def format(self, record):
    try:
      self.current_record = record
      data = self._parse_record(record)

      # 检查是否已经是格式化过的消息
      if isinstance(data, dict) and 'msg' in data:
        msg = data['msg']
        if isinstance(msg, str) and ' | ' in msg:
          # 如果消息已经包含格式化的内容，直接返回原始消息
          return msg

      # 提取日志信息
      module = self._extract_module(data)
      level = data.get('level', 'INFO')
      log_level = data.get('levelnum', logging.INFO)
      message = self._format_message(data)
      timestamp = self._get_timestamp(data)

      # 检查是否应该记录
      message_size = len(str(record)) if record else 0
      if not self._should_log(module, log_level, message_size):
        return None

      # 简化格式，移除线程信息和调用位置
      return f"{timestamp} | {level:7s} | {module:15s} | {message}"

    except Exception as e:
      return f"Logging error: {str(e)} - Record: {str(record)[:200]}"
    finally:
      self.current_record = None

  def _parse_record(self, record):
    if isinstance(record, str):
      try:
        return json.loads(record)
      except json.JSONDecodeError:
        return {'msg': record}
    return record

  def _extract_module(self, data):
    module = data.get('module')
    if not module:
      msg = data.get('msg', {})
      if isinstance(msg, dict):
        module = msg.get('module')
      ctx = data.get('ctx', {})
      if isinstance(ctx, dict):
        module = module or ctx.get('module')
      if not module and 'filename' in data:
        filename = data['filename']
        module = Path(filename).stem
        if '/' in module:
          module = module.split('/')[-1]
    return module or 'unknown'

  def _format_message(self, data):
    try:
      msg = data.get('msg', data)
      if isinstance(msg, (dict, list)):
        return json.dumps(msg, ensure_ascii=False)
      return str(msg)
    except Exception:
      return str(data)

  def _get_timestamp(self, data):
    try:
      return time.strftime('%Y-%m-%d %H:%M:%S.%f',
                          time.localtime(data.get('created', time.time())))
    except Exception:
      return time.strftime('%Y-%m-%d %H:%M:%S.%f')

def create_log_handler():
  """创建日志处理器"""
  try:
    Path(SWAGLOG_DIR).mkdir(parents=True, exist_ok=True)
    date_str = time.strftime('%Y%m%d_%H%M%S')
    session_id = hex(int(time.time()))[2:]

    log_file = os.path.join(
      SWAGLOG_DIR,
      f"swaglog.{date_str}.{session_id}.log"  # 主文件使用
    )

    handler = SwaglogRotatingFileHandler(
      base_filename=log_file,
      max_bytes=512*1024,
      backup_count=500
    )
    return handler
  except Exception as e:
    print(f"Error creating log handler: {str(e)}")
    return None

def main() -> NoReturn:
  params = Params()

  # 创建主日志处理器
  log_handler = create_log_handler()
  if not log_handler:
    return

  log_handler.setFormatter(FilteredLogFormatter(None))
  log_level = logging.INFO

  # ZMQ 设置
  ctx = zmq.Context.instance()
  sock = ctx.socket(zmq.PULL)
  sock.bind("ipc:///tmp/logmessage")

  # 消息发布器
  log_message_sock = messaging.pub_sock('logMessage')
  error_log_message_sock = messaging.pub_sock('errorLogMessage')

  try:
    while True:
      try:
        # 接收消息
        dat = b''.join(sock.recv_multipart())
        level = int(dat[0])
        record = dat[1:].decode("utf-8")

        # 解析记录
        try:
            data = json.loads(record)
        except json.JSONDecodeError:
            # 如果解析失败，说明可能是已格式化的字符串
            data = {'msg': record}

        # 获取自定义日志目录
        custom_log_dir = data.get('custom_log_dir') if isinstance(data, dict) else None

        # 格式化记录
        formatted_record = log_handler.formatter.format(record)
        if not formatted_record:
            continue

        # 处理主日志
        if level >= log_level:
          try:
            log_handler.emit(formatted_record)
          except Exception as e:
            print(f"Error emitting log: {e}")

        # 处理自定义目录日志
        if custom_log_dir:
          try:
            module = log_handler.formatter._extract_module(data)
            Path(custom_log_dir).mkdir(parents=True, exist_ok=True)

            date_str = time.strftime('%Y%m%d_%H%M%S')
            session_id = hex(int(time.time()))[2:]

            log_file = os.path.join(
              custom_log_dir,
              f"{module}.{date_str}.{session_id}.log"  # 主文件
            )

            with open(log_file, 'a', encoding='utf-8') as f:
              f.write(formatted_record + "\n")
          except Exception as e:
            print(f"Error writing to custom log: {e}")

        # 检查消息大小
        if len(formatted_record) > 2*1024*1024:
          print(f"WARNING: log too big to publish: {len(formatted_record)} bytes")
          continue

        # 发布消息
        try:
          msg = messaging.new_message()
          msg.logMessage = formatted_record
          log_message_sock.send(msg.to_bytes())

          if level >= logging.ERROR:
            msg = messaging.new_message()
            msg.errorLogMessage = formatted_record
            error_log_message_sock.send(msg.to_bytes())
        except zmq.error.Again:
          print("WARNING: Message queue full, dropping message")
        except Exception as e:
          print(f"Error sending message: {e}")

      except Exception as e:
        print(f"Error processing message: {e}")
        time.sleep(0.1)

  finally:
    sock.close()
    ctx.term()
    try:
      log_handler.close()
    except ValueError:
      pass

if __name__ == "__main__":
  main()
