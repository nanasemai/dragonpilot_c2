#!/usr/bin/env python3
import datetime
import time
import os
import signal
import subprocess
import sys
import traceback
from typing import List, Tuple, Union

from cereal import log
import cereal.messaging as messaging
import openpilot.selfdrive.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.selfdrive.boardd.set_time import set_time
from openpilot.system.hardware import HARDWARE, PC
from openpilot.selfdrive.manager.helpers import unblock_stdout, write_onroad_params
from openpilot.selfdrive.manager.process import ensure_running
from openpilot.selfdrive.manager.process_config import managed_processes,ensure_dependencies
from openpilot.selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
  get_normalized_origin, terms_version, training_version, \
  is_tested_branch, is_release_branch, get_commit_date

import json
from openpilot.selfdrive.car.fingerprints import all_known_cars, all_legacy_fingerprint_cars


def manager_init() -> None:
  # update system time from panda
  set_time(cloudlog)

  # save boot log
  subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if is_release_branch():
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LastValidTime", "0"),  # 添加最后有效时间参数
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("DisableUpdates", "1"),
    ("DPTimeZone", "Asia/Shanghai"),
    ("DPDEVMODE", "0"),
    ("IsMetric", "1"),
    ("dp_no_gps_ctrl", "0"),
    ("dp_no_fan_ctrl", "1"),
    ("dp_logging", "0"),
    ("dp_0813", "1"),
    ("dp_lat_controller", "0"), # Lateral Controller
    # dp addition
    ("dp_alka", "0"),
    ("dp_mapd", "0"),
    ("dp_lat_lane_priority_mode", "0"),
    ("dp_device_auto_shutdown", "0"),
    ("dp_device_auto_shutdown_in", "30"),
    ("dp_toyota_sng", "0"),
    ("dp_toyota_enhanced_bsm", "0"),
    ("dp_toyota_auto_lock", "0"),
    ("dp_toyota_auto_unlock", "0"),
    ("dp_device_audible_alert_mode", "0"),
    ("dp_device_disable_temp_check", "0"),
    ("dp_car_dashcam_mode_removal", "0"),
    ("dp_device_enable_comma_registration", "0"),
    ("dp_long_accel_profile", "0"),
    ("dp_long_use_df_tune", "0"),
    ("dp_long_de2e", "0"),
    ("dp_mapd_vision_turn_control", "0"),
    ("dp_hkg_min_steer_speed_bypass", "0"),
    ("dp_lat_lane_priority_mode_speed_based", "0"),
    ("dp_long_use_krkeegen_tune", "0"),
    ("dp_toyota_zss", "0"),
    ("dp_long_accel_btn", "0"),
    ("dp_long_personality_btn", "0"),
    ("dp_lat_lane_change_assist_speed", "20"),
    ("dp_vag_timebomb_bypass", "0"),
    ("dp_otisserv", "0"),
    ("dp_long_missing_lead_warning", "0"),
    ("dp_on_road_dashcam", "0"),
    ("dp_lateral_road_edge_detected", "0"),
    ("dp_use_nnff", "0"),
    ("dp_use_nnff_lite", "0"),
    ("NNFFModelName", ""),
    ("dp_log_level", "0"),  # 添加日志级别默认参数
    ("dp_device_mode", "1"),  # 设备运行模式: 0-节能 1-普通 2-性能
    ("dp_show_date_time", "1"),    # 是否显示时间: 0-不显示 1-显示
    # 行车记录仪相关参数
    ("dp_dashcam_quality", "medium"),  # 视频质量：低/中/高
    ("dp_dashcam_duration", "180"),    # 单个视频时长（秒）
    ("dp_dashcam_kept_hours", "15"),   # 视频保留时长（小时）
    ("dp_torqued_override", "0"),
    ("dp_torque_lat_accel_factor", "250"),
    ("dp_torque_friction", "1"),
    ("dp_gpxd", "0"),
    ("dp_fleet_fileserv", "0"),
    ("dp_dev_ui_info", "0"),
    ("dp_upload_on", "0"),
    ("dp_device_display_off_mode", "0"),
    # 添加换道中止检查参数
    ("dp_lat_lane_change_abort_check", "0"),
    ("dp_alka_torque_check", "0"),  # ALKA力矩检查开关: 0-关闭 1-开启
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  params.put("dp_car_list", get_support_car_list())

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit())
  params.put("GitCommitDate", get_commit_date())
  params.put("GitBranch", get_short_branch())
  params.put("GitRemote", get_origin())
  params.put_bool("IsTestedBranch", is_tested_branch())
  params.put_bool("IsReleaseBranch", is_release_branch())

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = get_normalized_origin() # Needed for swaglog
  os.environ['GIT_BRANCH'] = get_short_branch() # Needed for swaglog
  os.environ['GIT_COMMIT'] = get_commit() # Needed for swaglog

  if not is_dirty():
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=get_version(),
                       origin=get_normalized_origin(),
                       branch=get_short_branch(),
                       commit=get_commit(),
                       dirty=is_dirty(),
                       device=HARDWARE.get_device_type())

def manager_prepare() -> None:
  # 按优先级对进程排序
  priority_processes = {
    'critical': ['boardd', 'ubloxd', 'gpsd'],  # 系统关键进程
    'high': ['controlsd', 'plannerd', 'radard'],  # 控制相关进程
    'medium': ['modeld', 'locationd', 'paramsd'],  # 模型和定位进程
    'low': ['uploader', 'logmessaged', 'logcatd']  # 日志和上传进程
  }

  prepared = set()
  # 按优先级准备进程
  for priority in ['critical', 'high', 'medium', 'low']:
    for proc_name in priority_processes[priority]:
      if proc_name in managed_processes and proc_name not in prepared:
        # 确保依赖进程已准备
        if ensure_dependencies(proc_name, prepared):
          managed_processes[proc_name].prepare()
          prepared.add(proc_name)

  # 准备其他未分类进程
  for p in managed_processes.values():
    if p.name not in prepared:
      managed_processes[p.name].prepare()
      prepared.add(p.name)

def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")

def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  # 优化环境变量输出格式
  env_info = {
    "系统环境": {
      "PATH": os.environ.get("PATH", ""),
      "PYTHONPATH": os.environ.get("PYTHONPATH", ""),
      "BASEDIR": os.environ.get("BASEDIR", "")
    },
    "设备信息": {
      "DONGLE_ID": os.environ.get("DONGLE_ID", ""),
      "DEVICE": os.environ.get("DEVICE", "")
    },
    "版本信息": {
      "GIT_ORIGIN": os.environ.get("GIT_ORIGIN", ""),
      "GIT_BRANCH": os.environ.get("GIT_BRANCH", ""),
      "GIT_COMMIT": os.environ.get("GIT_COMMIT", ""),
      "CLEAN": os.environ.get("CLEAN", "0")
    }
  }
  cloudlog.info(env_info)

  params = Params()

  ignore: List[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")

  if not params.get_bool("dp_logging"):
    ignore += ["logcatd", "proclogd", "loggerd"]
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  if not params.get_bool("dp_mapd"):
    ignore += ["mapd"]

  if not params.get_bool("dp_gpxd"):
    ignore += ["gpxd"]

  if params.get_bool("dp_no_gps_ctrl"):
    ignore += ["ubloxd", "gpx_uploader", "gpxd", "mapd"]

  if not params.get_bool("dp_fleet_fileserv"):
    ignore += ["fleet_manager"]

  if not params.get_bool("dp_otisserv"):
    ignore += ["otisserv"]

  if not params.get_bool("dp_on_road_dashcam"):
      ignore += ["systemd"]

  if not params.get_bool("dp_upload_on"):
    ignore += ["uploader"]

  #add by nana
  ignore += ["manage_athenad"]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  last_network_type = log.DeviceState.NetworkType.none

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  started_prev = False
  last_time_save = 0  # 添加时间保存计数器
  while True:
    sm.update(1000)

    started = sm['deviceState'].started
    current_time = int(time.time())

    # 检查网络状态变化时更新时间
    if sm.updated['deviceState']:
      current_network = sm['deviceState'].networkType
      if current_network != last_network_type and current_network != log.DeviceState.NetworkType.none:
        set_time(cloudlog)
      last_network_type = current_network

    # 在以下情况保存时间：
    # 1. 每5分钟保存一次基础时间
    # 2. 网络连接成功时
    # 3. 进入或退出行驶状态时
    if (current_time - last_time_save >= 300 or  # 5分钟
        (sm.updated['deviceState'] and sm['deviceState'].networkType != last_network_type) or
        started != started_prev):
      try:
        params.put("LastValidTime", str(current_time))
        last_time_save = current_time
      except Exception as e:
        cloudlog.warning(f"保存系统时间失败: {str(e)}")

    # 添加进程状态监控
    process_states = {}
    for p in managed_processes.values():
      if p.proc is not None:
        process_states[p.name] = {
          'alive': p.proc.is_alive(),
          'exitcode': p.proc.exitcode if not p.proc.is_alive() else None,
          'restart_count': getattr(p, 'restart_count', 0)
        }

    # 记录异常进程状态
    for name, state in process_states.items():
      if not state['alive']:
        cloudlog.error(f"Process {name} died with exitcode {state['exitcode']}")
        if state['restart_count'] > 3:
          cloudlog.error(f"Process {name} restarted too many times")

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives boardd's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot", "dp_reset_conf"):
      if params.get_bool(param):
        if param == "dp_reset_conf":
          os.system("rm -fr /data/params/d/dp_*")
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break

def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()

def get_support_car_list():
  cars = dict({"cars": []})
  list = []
  for car in all_known_cars():
    list.append(str(car))

  for car in all_legacy_fingerprint_cars():
    name = str(car)
    if name not in list:
      list.append(name)
  cars["cars"] = sorted(list)
  return json.dumps(cars)

if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
