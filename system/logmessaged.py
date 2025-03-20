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
        'modeld'
      }
      self.reduced_modules = {
        'boardd': logging.WARNING,
        'logmessaged': logging.WARNING,
        'camerad': logging.WARNING,
        'ui': logging.WARNING
      }
      self.current_record = None

    def _should_log(self, module, log_level):
      if 'gpu' in module.lower() or module == 'modeld':
        return True
        
      try:
        message_size = len(self.current_record) if self.current_record else 0
        if message_size > 2*1024*1024:
          return log_level >= logging.ERROR
      except Exception:
        pass
        
      if log_level >= logging.ERROR:
        return True
      
      if module in self.critical_modules:
        return True
      
      if module in self.reduced_modules:
        return log_level >= self.reduced_modules[module]
      
      return True
    
    def fix_kv(self, k, v):
      if isinstance(v, (dict, list, tuple, str)):
        return k, v
      return k, str(v)
      
    def format(self, record):
      try:
        self.current_record = record
        
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

        module = v.get('module', None)
        if not module:
          try:
            if isinstance(v.get('msg'), dict) and 'module' in v['msg']:
              module = v['msg']['module']
            elif 'ctx' in v and isinstance(v['ctx'], dict):
              module = v['ctx'].get('module')
            elif 'filename' in v:
              module = v['filename'].split('/')[-1].replace('.py', '')
          except Exception:
            pass
        
        module = module or 'unknown'
        level = v.get('level', 'INFO')
        log_level = v.get('levelnum', logging.INFO)

        try:
          message_content = v.get('msg', v)
          mk, mv = self.fix_kv('msg', message_content)
          message = self._process_message(mv, module)
        except Exception as e:
          try:
            message = str(v)
          except Exception:
            message = "Unable to process log message"

        try:
          timestamp = time.strftime('%Y-%m-%d %H:%M:%S',
                                time.localtime(v.get('created', time.time())))
        except Exception:
          timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

        formatted_log = f"{timestamp} | {level:7s} | {module:15s} | {message}"
        
        if not self._should_log(module, log_level):
          return None

        return formatted_log

      except Exception as e:
        return f"Logging error: {str(e)} - Record: {str(record)[:200]}"
      finally:
        self.current_record = None

    def _process_message(self, message, module):
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
      try:
        dat = b''.join(sock.recv_multipart())
        level = int(dat[0])
        record = dat[1:].decode("utf-8")

        formatted_record = log_handler.formatter.format(record)
        if formatted_record:
          if level >= log_level:
            try:
              log_handler.emit(formatted_record)
            except Exception as e:
              print(f"Error emitting log: {e}")

          if len(formatted_record) > 2*1024*1024:
            print(f"WARNING: log too big to publish: {len(formatted_record)} bytes")
            continue

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