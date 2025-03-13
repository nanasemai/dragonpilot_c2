#!/usr/bin/env python3
import zmq
import json
import time
import logging
from typing import NoReturn

import cereal.messaging as messaging
from openpilot.common.logging_extra import SwagLogFileFormatter
from openpilot.common.swaglog import get_file_handler
from openpilot.common.params import Params

def main() -> NoReturn:
  params = Params()
  log_handler = get_file_handler()

  class FilteredLogFormatter(SwagLogFileFormatter):
    def __init__(self, swaglogger=None):
      super().__init__(swaglogger)
      self.critical_modules = {
        'controlsd', 'pandad', 'plannerd', 'radard',
        'thermald', 'uploader', 'manager', 'locationd',
        'modeld'  # 将 modeld 加入关键模块
      }
      self.reduced_modules = {
        'boardd': logging.WARNING,
        'logmessaged': logging.WARNING,
        'camerad': logging.WARNING,
        'ui': logging.WARNING
      }

    def _should_log(self, module, log_level):
      # GPU 相关日志始终记录
      if 'gpu' in str(record).lower() or module == 'modeld':
        return True
        
      # 只记录大于2MB的错误日志
      if len(str(record)) > 2*1024*1024:
        return log_level >= logging.ERROR
        
      # 其他逻辑保持不变
      if log_level >= logging.ERROR:
        return True
      
      if module in self.critical_modules:
        return True
      
      if module in self.reduced_modules:
        return log_level >= self.reduced_modules[module]
      
      return True
    
    def fix_kv(self, k, v):
      """处理键值对，返回格式化后的键和值"""
      if isinstance(v, dict):
        return k, v
      elif isinstance(v, list):
        return k, v
      elif isinstance(v, tuple):
        return k, v
      elif isinstance(v, str):
        return k, v
      else:
        return k, str(v)
      
    def format(self, record):
      try:
        # 处理字符串类型的记录
        if isinstance(record, str):
          try:
            v = json.loads(record)
          except json.JSONDecodeError:
            return record
        else:
          try:
            v = self.format_dict(record)
          except Exception as e:
            return f"Format error: {str(e)} - Raw record: {str(record)}"

        # 获取日志信息
        module = v.get('module', None)
        if not module:
          # 尝试从消息内容中提取模块名
          try:
            if isinstance(v.get('msg'), dict) and 'module' in v['msg']:
              module = v['msg']['module']
            elif 'ctx' in v and isinstance(v['ctx'], dict):
              module = v['ctx'].get('module')
            elif 'filename' in v:
              # 从文件名提取模块名
              module = v['filename'].split('/')[-1].replace('.py', '')
          except Exception:
            pass
        
        # 如果仍然无法获取模块名，使用默认值
        module = module or 'unknown'
        level = v.get('level', 'INFO')
        log_level = v.get('levelnum', logging.INFO)

        # 处理消息内容
        try:
          # 获取消息内容，如果没有msg键，则使用整个记录
          message_content = v.get('msg', v)
          mk, mv = self.fix_kv('msg', message_content)
          message = self._process_message(mv, module)
        except Exception as e:
          # 如果处理失败，尝试直接使用原始记录
          try:
            message = str(v)
          except Exception:
            message = "Unable to process log message"

        # 格式化时间戳
        try:
          timestamp = time.strftime('%Y-%m-%d %H:%M:%S',
                                time.localtime(v.get('created', time.time())))
        except Exception:
          timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

        # 先格式化日志
        formatted_log = f"{timestamp} | {level:7s} | {module:15s} | {message}"
        
        # 检查是否需要记录
        if not self._should_log(module, log_level):
          return None

        return formatted_log

      except Exception as e:
        # 如果发生任何错误，返回基本错误信息
        return f"Logging error: {str(e)} - Record: {str(record)[:200]}"

    def _process_message(self, message, module):
      """处理消息内容"""
      if isinstance(message, (dict, list)):
        return json.dumps(message, ensure_ascii=False)
      return str(message)

  log_handler.setFormatter(FilteredLogFormatter(None))
  log_level = logging.INFO

  ctx = zmq.Context.instance()
  sock = ctx.socket(zmq.PULL)
  sock.bind("ipc:///tmp/logmessage")

  log_message_sock = messaging.pub_sock('logMessage')
  error_log_message_sock = messaging.pub_sock('errorLogMessage')

  try:
    while True:
      dat = b''.join(sock.recv_multipart())
      level = int(dat[0])
      record = dat[1:].decode("utf-8")

      # 格式化日志记录
      formatted_record = log_handler.formatter.format(record)
      if formatted_record:
        if level >= log_level:
          log_handler.emit(formatted_record)

        if len(formatted_record) > 2*1024*1024:
          print("WARNING: log too big to publish", len(formatted_record))
          print(formatted_record[:100])
          continue

        msg = messaging.new_message()
        msg.logMessage = formatted_record
        log_message_sock.send(msg.to_bytes())

        if level >= logging.ERROR:
          msg = messaging.new_message()
          msg.errorLogMessage = formatted_record
          error_log_message_sock.send(msg.to_bytes())

  finally:
    sock.close()
    ctx.term()

    try:
      log_handler.close()
    except ValueError:
      pass

if __name__ == "__main__":
  main()