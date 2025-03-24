"""Install exception handler for process crash."""
import sentry_sdk
from enum import Enum
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params
from openpilot.selfdrive.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_branch, get_commit, get_origin, get_version, \
                              is_comma_remote, is_dirty, is_tested_branch

import os
import traceback
import requests
from cereal import car
from datetime import datetime

class SentryProject(Enum):
  SELFDRIVE = "https://980a0cba712a4c3593c33c78a12446e1@o273754.ingest.sentry.io/1488600"
  SELFDRIVE_NATIVE = "https://980a0cba712a4c3593c33c78a12446e1@o273754.ingest.sentry.io/1488600"

CRASHES_DIR = '/data/media/0/crash_logs'
ENABLE_REMOTE_LOGGING = False

ret = car.CarParams.new_message()
candidate = ret.carFingerprint

params = Params()
try:
  dongle_id = params.get("DongleId").decode('utf8')
except AttributeError:
  dongle_id = "None"
try:
  gitname = Params().get("GithubUsername", encoding='utf-8')
except Exception:
  gitname = ""
try:
  ip = requests.get('https://checkip.amazonaws.com/').text.strip()
except Exception:
  ip = "255.255.255.255"
error_tags = {'dirty': is_dirty(), 'dongle_id': dongle_id, 'branch': get_branch(), 
              'remote': get_origin(), 'fingerprintedAs': candidate, 'gitname': gitname}

try:
  cached_params = params.get("CarParams")
  if cached_params is not None:
    cached_params = car.CarParams.from_bytes(cached_params)
    car_name = cached_params.carFingerprint
  else:
    car_name = "None"
except Exception:
  car_name = "None"

def save_exception(exc_text):
  # 在函数开始处声明全局变量
  global CRASHES_DIR
  
  # 使用 os.makedirs 的 exist_ok 参数，避免竞态条件
  try:
    os.makedirs(CRASHES_DIR, exist_ok=True)
  except Exception as e:
    print(f"创建崩溃日志目录失败: {str(e)}")
    # 如果无法创建目录，尝试使用备用路径
    backup_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'crash_logs')
    try:
      os.makedirs(backup_dir, exist_ok=True)
      CRASHES_DIR = backup_dir
    except Exception:
      print("无法创建备用崩溃日志目录")
      return None

  timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
  # 使用更规范的文件命名方式，包含更多信息
  log_file = os.path.join(CRASHES_DIR, f'crash_{HARDWARE.get_device_type()}_{timestamp}.log')
  
  system_info = {
    'version': get_version(),
    'branch': get_branch(),
    'commit': get_commit(),
    'dirty': is_dirty(),
    'device': HARDWARE.get_device_type(),
    'dongle_id': dongle_id,
    'car_model': car_name
  }
  
  with open(log_file, 'w') as f:
    f.write("=== System Info ===\n")
    for k, v in system_info.items():
      f.write(f"{k}: {v}\n")
    f.write("\n=== Exception ===\n")
    f.write(exc_text)
  
  print(f'Logged crash to {log_file}')
  return log_file

def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})
  
  if ENABLE_REMOTE_LOGGING:
    with sentry_sdk.configure_scope() as scope:
      scope.set_extra("tombstone_fn", fn)
      scope.set_extra("tombstone", contents)
      sentry_sdk.capture_message(message=message)
      sentry_sdk.flush()

def capture_exception(*args, **kwargs) -> None:
  save_exception(traceback.format_exc())
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  if ENABLE_REMOTE_LOGGING:
    try:
      sentry_sdk.capture_exception(*args, **kwargs)
      sentry_sdk.flush()
    except Exception:
      cloudlog.exception("sentry exception")

def bind_user(**kwargs) -> None:
  if ENABLE_REMOTE_LOGGING:
    sentry_sdk.set_user(kwargs)

def capture_warning(warning_string):
  cloudlog.warning(warning_string)
  if ENABLE_REMOTE_LOGGING:
    bind_user(id=dongle_id, ip_address=ip, name=gitname)
    sentry_sdk.capture_message(warning_string, level='warning')

def capture_info(info_string):
  cloudlog.info(info_string)
  if ENABLE_REMOTE_LOGGING:
    bind_user(id=dongle_id, ip_address=ip, name=gitname)
    sentry_sdk.capture_message(info_string, level='info')

def set_tag(key: str, value: str) -> None:
  if ENABLE_REMOTE_LOGGING:
    sentry_sdk.set_tag(key, value)

def init(project: SentryProject) -> None:
  if not ENABLE_REMOTE_LOGGING:
    cloudlog.info("Remote logging disabled")
    return

  env = "release" if is_tested_branch() else "master"
  dongle_id = Params().get("DongleId", encoding='utf-8')
  gitname = Params().get("GithubUsername", encoding='utf-8')

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))
  else:
    sentry_sdk.utils.MAX_STRING_LENGTH = 8192

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_user({"gitname": gitname})
  sentry_sdk.set_tag("dirty", is_dirty())
  sentry_sdk.set_tag("origin", get_origin())
  sentry_sdk.set_tag("branch", get_branch())
  sentry_sdk.set_tag("commit", get_commit())
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())
  sentry_sdk.set_tag("model", car_name)

  if project == SentryProject.SELFDRIVE:
    sentry_sdk.Hub.current.start_session()
