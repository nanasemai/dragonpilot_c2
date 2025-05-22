import os
import time
import subprocess
from functools import wraps
from pathlib import Path

from flask import render_template, request, session
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths
from openpilot.selfdrive.loggerd.uploader import listdir_by_creation

from tools.lib.route import SegmentName

# path to sunnypilot screen recordings and error logs
if PC:
  SCREENRECORD_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "dashcam", "")
  ERROR_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "crash_logs", "")
  C2_LOGS_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "c2_logs", "")
  GPX_RECORD_PATH = os.path.join(str(Path.home()), ".comma", "media", "0", "gpx_logs", "")
  PIN_PATH = os.path.join(str(Path.home()), ".comma", "otp", "")
else:
  SCREENRECORD_PATH = "/data/media/0/dashcam/"
  ERROR_LOGS_PATH = "/data/media/0/crash_logs/"
  C2_LOGS_PATH = "/data/media/0/c2_logs/"
  GPX_RECORD_PATH = "/data/media/0/gpx_logs/"
  PIN_PATH = "/data/otp/"


def login_required(f):
  @wraps(f)
  def decorated_route(*args, **kwargs):
    if not session.get("logged_in"):
      session["previous_page"] = request.url
      return render_template("login.html")
    return f(*args, **kwargs)
  return decorated_route

def is_valid_segment(segment):
  try:
    segment_to_segment_name(Paths.log_root(), segment)
    return True
  except AssertionError:
    return False


def segment_to_segment_name(data_dir, segment):
  fake_dongle = "ffffffffffffffff"
  return SegmentName(str(os.path.join(data_dir, fake_dongle + "|" + segment)))


def all_segment_names():
  segments = []
  for segment in listdir_by_creation(Paths.log_root()):
    try:
      segments.append(segment_to_segment_name(Paths.log_root(), segment))
    except AssertionError:
      pass
  return segments


def all_routes():
  segment_names = all_segment_names()
  route_names = [segment_name.route_name for segment_name in segment_names]
  route_times = [route_name.time_str for route_name in route_names]
  unique_routes = list(dict.fromkeys(route_times))
  return sorted(unique_routes, reverse=True)


def segments_in_route(route):
  segment_names = [segment_name for segment_name in all_segment_names() if segment_name.time_str == route]
  segments = [segment_name.time_str + "--" + str(segment_name.segment_num) for segment_name in segment_names]
  return segments


def ffmpeg_mp4_concat_wrap_process_builder(file_list, cameratype, chunk_size=1024*512):
  command_line = ["ffmpeg"]
  if not cameratype == "qcamera":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", "concat:" + file_list]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if not cameratype == "qcamera":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE,
    bufsize=chunk_size
  )


def ffmpeg_mp4_wrap_process_builder(filename):
  """Returns a process that will wrap the given filename
     inside a mp4 container, for easier playback by browsers
     and other devices. Primary use case is streaming segment videos
     to the vidserver tool.
     filename is expected to be a pathname to one of the following
       /path/to/a/qcamera.ts
       /path/to/a/dcamera.hevc
       /path/to/a/ecamera.hevc
       /path/to/a/fcamera.hevc
  """
  basename = filename.rsplit("/")[-1]
  extension = basename.rsplit(".")[-1]
  command_line = ["ffmpeg"]
  if extension == "hevc":
    command_line += ["-f", "hevc"]
  command_line += ["-r", "20"]
  command_line += ["-i", filename]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  if extension == "hevc":
    command_line += ["-vtag", "hvc1"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )


def ffplay_mp4_wrap_process_builder(file_name):
  command_line = ["ffmpeg"]
  command_line += ["-i", file_name]
  command_line += ["-c", "copy"]
  command_line += ["-map", "0"]
  command_line += ["-f", "mp4"]
  command_line += ["-movflags", "empty_moov"]
  command_line += ["-"]
  return subprocess.Popen(
    command_line, stdout=subprocess.PIPE
  )
def get_file_info(full_path, name, base_path=""):
    info = {
        "name": name,
        "type": "file",
        "size": "",
        "mtime": "",
        "path": os.path.relpath(full_path, base_path) if base_path else name
    }
    
    try:
        stat = os.stat(full_path)
        info["mtime"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime))
        
        if os.path.isdir(full_path):
            info["type"] = "directory"
        else:
            size = stat.st_size
            if size < 1024:
                info["size"] = f"{size} B"
            elif size < 1024*1024:
                info["size"] = f"{size/1024:.1f} KB"
            else:
                info["size"] = f"{size/(1024*1024):.1f} MB"
    except Exception:
        pass
    
    return info

def list_files(path, single=False):
    if not os.path.exists(path):
        return []
        
    files = []
    try:
        entries = os.listdir(path) if single else listdir_by_creation(path)
        
        for name in entries:
            full_path = os.path.join(path, name)
            info = get_file_info(full_path, name, path)
            files.append(info)
        
        return sorted(files, key=lambda x: x["name"], reverse=True)
    except Exception:
        return []


PARAMS_PATH = os.path.join(str(Path.home()), ".comma", "params", "") if PC else "/data/params/d/"

def read_param(param_name):
    try:
        with open(os.path.join(PARAMS_PATH, param_name), 'r') as f:
            return f.read().strip()
    except Exception:
        return None

def write_param(param_name, value):
    try:
        with open(os.path.join(PARAMS_PATH, param_name), 'w') as f:
            f.write(value)
        return True
    except Exception:
        return False

def list_params():
    try:
        all_params = os.listdir(PARAMS_PATH)
        return [f for f in all_params if os.path.isfile(os.path.join(PARAMS_PATH, f)) and f in PARAM_DESCRIPTIONS]
    except Exception:
        return []


PARAM_DESCRIPTIONS = {
    "dp_0813": "开启0813模型",
    "dp_no_gps_ctrl": "禁用GPS控制功能",
    "dp_no_fan_ctrl": "禁用风扇控制功能", 
    "dp_logging": "日志记录开关",
    "dp_device_no_ir_ctrl": "禁用红外控制功能",
    "dp_alka": "ALKA模式开关",
    "dp_mapd": "地图数据开关",
    "dp_lat_lane_priority_mode": "车道优先级模式",
    "dp_device_auto_shutdown": "设备自动关机设置",
    "dp_device_auto_shutdown_in": "设备自动关机时间设置",
    "dp_toyota_sng": "丰田停止前进功能",
    "dp_toyota_enhanced_bsm": "丰田增强型盲点监测",
    "dp_toyota_auto_lock": "丰田自动锁门功能",
    "dp_toyota_auto_unlock": "丰田自动解锁功能",
    "dp_device_audible_alert_mode": "设备声音警报模式",
    "dp_device_disable_temp_check": "禁用设备温度检查",
    "dp_car_dashcam_mode_removal": "强制移除Dashcam模式",
    "dp_long_de2e": "纵向DE2E控制",
    "dp_mapd_vision_turn_control": "地图视觉转向控制",
    "dp_hkg_min_steer_speed_bypass": "现代/起亚最小转向速度绕过",
    "dp_lat_lane_priority_mode_speed_based": "基于速度的车道优先级",
    "dp_long_use_krkeegen_tune": "使用Krkeegen调校参数",
    "dp_toyota_zss": "丰田ZSS转向传感器支持",
    "dp_long_accel_btn": "纵向加速按钮设置",
    "dp_long_personality_btn": "纵向个性按钮设置",
    "dp_vag_timebomb_bypass": "VAG车型横向控制时间限制绕过",
    "dp_lat_lane_change_assist_speed": "换道辅助激活速度",
    "dp_long_missing_lead_warning": "前车丢失警告",
    "dp_on_road_dashcam": "行车记录仪开关",
    "dp_lateral_road_edge_detected": "道路边缘检测",
    "dp_use_nnff": "神经网络前馈控制",
    "dp_use_nnff_lite": "简化版神经网络控制",
    "dp_dashcam_quality": "行车记录仪画质设置",
    "dp_dashcam_duration": "行车记录仪录制时长",
    "dp_dashcam_kept_hours": "行车记录仪保留时长",
    "dp_torqued_override": "手动实时调教开关",
    "dp_torque_lat_accel_factor": "最大横向加速度设置",
    "dp_torque_friction": "转向摩擦力设置",
    "dp_gpxd": "GPX数据记录开关",
    "dp_dev_ui_info": "开发者UI信息显示设置",
    "dp_device_display_off_mode": "设备显示屏关闭模式",
    "dp_lat_lane_change_abort_check": "换道中止检查",
    "dp_device_go_off_road": "设备离线模式",
    "dp_lateral_camera_offset": "相机偏移量设置",
    "dp_lateral_path_offset": "路径偏移量设置",
    "dp_lateral_torque_kp": "PID实时P值设置",
    "dp_lateral_torque_ki": "PID实时I值设置",
    "dp_disable_gps": "禁用GPS功能", 
    "dp_lon_acm": "纵向ACM控制",
    "dp_lon_acm_downhill": "下坡纵向ACM控制",
    "dp_lead_start_alert_threshold": "前车起步检测阈值(m/s)",
    "dp_lead_stop_time_threshold": "前车停止时间阈值(s)",
    "dp_lat_use_siglin": "BYD Siglin模式开关",
    "BydModifiedStockLong": "BYD改进版原车纵向控制",
    "BydUseRadar": "BYD使用雷达进行纵向控制"
}

def get_param_description(param_name):
    return PARAM_DESCRIPTIONS.get(param_name, "未定义参数")


PARAM_VALIDATORS = {
    "dp_log_level": lambda v: v.isdigit() and 0 <= int(v) <= 4,
    "dp_device_mode": lambda v: v.isdigit() and 0 <= int(v) <= 2,
    "dp_dev_ui_info": lambda v: v.isdigit() and 0 <= int(v) <= 3,
    "dp_dashcam_quality": lambda v: True,
    "dp_dashcam_duration": lambda v: True, 
    "dp_dashcam_kept_hours": lambda v: True,
    "dp_alka": lambda v: v in ["0", "1"],
    "dp_use_nnff": lambda v: v in ["0", "1"],
    "dp_use_nnff_lite": lambda v: v in ["0", "1"],
    "dp_torqued_override": lambda v: v in ["0", "1"],
    "dp_show_date_time": lambda v: v in ["0", "1"],
    "dp_lat_controller": lambda v: True, 
    "dp_long_accel_profile": lambda v: True,  
    "dp_lat_lane_priority_mode": lambda v: True, 
    "dp_device_auto_shutdown": lambda v: v in ["0", "1"],
    "dp_device_display_off_mode": lambda v: v in ["0", "1"],
    "dp_lateral_camera_offset": lambda v: True, 
    "dp_lateral_path_offset": lambda v: True, 
    "dp_lateral_torque_kp": lambda v: True, 
    "dp_lateral_torque_ki": lambda v: True, 
    "dp_disable_gps": lambda v: v in ["0", "1"],
    "dp_lead_start_alert": lambda v: v in ["0", "1"],
    "dp_lead_start_alert_threshold": lambda v: True,
    "dp_lead_stop_time_threshold": lambda v: True, 
    "dp_lat_use_siglin": lambda v: v in ["0", "1"],
    "BydModifiedStockLong": lambda v: v in ["0", "1"],
    "BydUseRadar": lambda v: v in ["0", "1"]
}

def validate_param(param_name, value):
    if param_name in PARAM_VALIDATORS:
        return PARAM_VALIDATORS[param_name](value)
    return True  # 默认允许所有参数