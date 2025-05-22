#!/usr/bin/env python3
import os
import random
import secrets
import threading
import time
from flask import Flask, render_template, Response, request, send_from_directory, session, redirect, url_for, jsonify # 导入 jsonify
from openpilot.common.realtime import set_core_affinity
import openpilot.system.fleetmanager.helpers as fleet
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params # 导入 Params

# 导入 screenshot_server 需要的模块
import glob
import json
import socket
import subprocess

app = Flask(__name__)


@app.route("/")
@app.route("/index")
def home_page():
  return render_template("index.html")


@app.route("/login", methods=["POST"])
def login():
  #inputted_pin = request.form.get("pin")
  #with open(fleet.PIN_PATH + "otp.conf", "r") as file:
  #  correct_pin = file.read().strip()
  #
  #if inputted_pin == correct_pin:
    session["logged_in"] = True
    if "previous_page" in session:
      previous_page = session["previous_page"]
      session.pop("previous_page", None)
      return redirect(previous_page)
    else:
      return redirect(url_for("home_page"))
  #else:
  #  error_message = "Incorrect PIN. Please try again."
  #  return render_template("login.html", error=error_message)


@app.route("/footage/full/<cameratype>/<route>")
def full(cameratype, route):
  chunk_size = 1024 * 512  # 5KiB
  file_name = cameratype + (".ts" if cameratype == "qcamera" else ".hevc")
  vidlist = "|".join(Paths.log_root() + "/" + segment + "/" + file_name for segment in fleet.segments_in_route(route))

  def generate_buffered_stream():
    with fleet.ffmpeg_mp4_concat_wrap_process_builder(vidlist, cameratype, chunk_size) as process:
      for chunk in iter(lambda: process.stdout.read(chunk_size), b""):
        yield bytes(chunk)
  return Response(generate_buffered_stream(), status=200, mimetype='video/mp4')


@app.route("/footage/<cameratype>/<segment>")
def fcamera(cameratype, segment):
  if not fleet.is_valid_segment(segment):
    return render_template("error.html", error="invalid segment")
  file_name = Paths.log_root() + "/" + segment + "/" + cameratype + (".ts" if cameratype == "qcamera" else ".hevc")
  return Response(fleet.ffmpeg_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')


@app.route("/footage/<route>")
def route(route):
  if len(route) != 20:
    return render_template("error.html", error="route not found")

  if str(request.query_string) == "b''":
    query_segment = str("0")
    query_type = "qcamera"
  else:
    query_segment = (str(request.query_string).split(","))[0][2:]
    query_type = (str(request.query_string).split(","))[1][:-1]

  links = ""
  segments = ""
  for segment in fleet.segments_in_route(route):
    links += "<a href='"+route+"?"+segment.split("--")[2]+","+query_type+"'>"+segment+"</a><br>"
    segments += "'"+segment+"',"
  return render_template("route.html", route=route, query_type=query_type, links=links, segments=segments, query_segment=query_segment)


@app.route("/footage/")
@app.route("/footage")
def footage():
  return render_template("footage.html", rows=fleet.all_routes())


@app.route("/screenrecords/")
@app.route("/screenrecords")
def screenrecords():
  rows = fleet.list_files(fleet.SCREENRECORD_PATH, True)
  if not rows:
    return render_template("error.html", error="no screenrecords found at:<br><br>" + fleet.SCREENRECORD_PATH)
  return render_template("screenrecords.html", rows=rows, clip=rows[0])


@app.route("/screenrecords/<clip>")
def screenrecord(clip):
  return render_template("screenrecords.html", rows=fleet.list_files(fleet.SCREENRECORD_PATH), clip=clip)


@app.route("/screenrecords/play/pipe/<file>")
def videoscreenrecord(file):
  file_name = fleet.SCREENRECORD_PATH + file
  return Response(fleet.ffplay_mp4_wrap_process_builder(file_name).stdout.read(), status=200, mimetype='video/mp4')


@app.route("/screenrecords/download/<clip>")
def download_screenrecord(clip):
    try:
        full_path = os.path.abspath(os.path.join(fleet.SCREENRECORD_PATH, clip))
        if not full_path.startswith(os.path.abspath(fleet.SCREENRECORD_PATH)):
            return render_template("error.html", error="非法的路径访问")

        if not os.path.isfile(full_path):
            return render_template("error.html", error="文件不存在")

        return send_from_directory(os.path.dirname(full_path), os.path.basename(full_path), as_attachment=True)
    except Exception as e:
        return render_template("error.html", error=f"下载文件出错: {str(e)}")


# 保留原有的通用下载路由
@app.route("/download/<path:file_type>/<path:file_path>")
def download_any_file(file_type, file_path):
    try:
        base_path = {
            "screenrecords": fleet.SCREENRECORD_PATH,
            "error_logs": fleet.ERROR_LOGS_PATH,
            "gpx_logs": fleet.GPX_RECORD_PATH,
            "c2_logs": fleet.C2_LOGS_PATH
        }.get(file_type)

        if not base_path:
            return render_template("error.html", error="不支持的文件类型")

        full_path = os.path.abspath(os.path.join(base_path, file_path))
        if not full_path.startswith(os.path.abspath(base_path)):
            return render_template("error.html", error="非法的路径访问")

        if not os.path.isfile(full_path):
            return render_template("error.html", error="文件不存在")

        return send_from_directory(os.path.dirname(full_path), os.path.basename(full_path), as_attachment=True)
    except Exception as e:
        return render_template("error.html", error=f"下载文件出错: {str(e)}")


@app.route("/about")
def about():
  return render_template("about.html")

# 添加错误日志路由
@app.route("/error_logs")
@app.route("/error_logs/")
def error_logs():
    try:
        if not os.path.exists(fleet.ERROR_LOGS_PATH):
            return render_template("error.html", error=f"目录不存在: {fleet.ERROR_LOGS_PATH}")
        return render_template("error_logs.html", rows=fleet.list_files(fleet.ERROR_LOGS_PATH))
    except Exception as e:
        return render_template("error.html", error=f"访问目录出错: {str(e)}")

@app.route("/error_logs/<file_name>")
def open_error_log(file_name):
    try:
        full_path = os.path.join(fleet.ERROR_LOGS_PATH, file_name)
        if not os.path.exists(full_path):
            return render_template("error.html", error="文件不存在")

        with open(full_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(full_path, 'r', encoding='gbk') as f:
            content = f.read()
    except Exception as e:
        return render_template("error.html", error=f"读取文件出错: {str(e)}")
    return render_template("error_log.html", file_name=file_name, file_content=content)

@app.route("/gpx_logs")
@app.route("/gpx_logs/")
def gpx_logs():
    try:
        if not os.path.exists(fleet.GPX_RECORD_PATH):
            return render_template("error.html", error=f"目录不存在: {fleet.GPX_RECORD_PATH}")
        return render_template("gpx_logs.html", rows=fleet.list_files(fleet.GPX_RECORD_PATH))
    except Exception as e:
        return render_template("error.html", error=f"访问目录出错: {str(e)}")

@app.route("/gpx_logs/<file_name>")
def open_gpx_log(file_name):
    try:
        with open(fleet.GPX_RECORD_PATH + file_name, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(fleet.ERROR_LOGS_PATH + file_name, 'r', encoding='gbk') as f:
            content = f.read()
    except Exception as e:
        return render_template("error.html", error=f"读取文件出错: {str(e)}")
    return render_template("error_log.html", file_name=file_name, file_content=content)  # 修正模板名

def generate_pin():
  if not os.path.exists(fleet.PIN_PATH):
    os.makedirs(fleet.PIN_PATH)
  pin = str(random.randint(100000, 999999))
  with open(fleet.PIN_PATH + "otp.conf", "w") as file:
    file.write(pin)


def schedule_pin_generate():
  pin_thread = threading.Thread(target=update_pin)
  pin_thread.start()


def update_pin():
  while True:
    generate_pin()
    time.sleep(30)

@app.route("/c2_logs")
@app.route("/c2_logs/")
def c2_logs():
    try:
        if not os.path.exists(fleet.C2_LOGS_PATH):
            return render_template("error.html", error=f"目录不存在: {fleet.C2_LOGS_PATH}")
        folders = []
        for entry in os.scandir(fleet.C2_LOGS_PATH):
            if entry.is_dir():
                info = fleet.get_file_info(entry.path, entry.name, fleet.C2_LOGS_PATH)
                folders.append(info)
        return render_template("c2_logs.html", rows=sorted(folders, key=lambda x: x["name"], reverse=True))
    except Exception as e:
        return render_template("error.html", error=f"访问目录出错: {str(e)}")

@app.route("/c2_logs/<path:folder_path>")
def open_c2_log(folder_path):
    try:
        full_path = os.path.abspath(os.path.join(fleet.C2_LOGS_PATH, folder_path))
        if not full_path.startswith(os.path.abspath(fleet.C2_LOGS_PATH)):
            return render_template("error.html", error="非法的路径访问")

        if os.path.isdir(full_path):
            files = []
            for entry in os.scandir(full_path):
                if entry.is_file():
                    info = fleet.get_file_info(entry.path, entry.name, os.path.dirname(full_path))
                    files.append(info)
            return render_template("c2_logs.html", rows=sorted(files, key=lambda x: x["name"], reverse=True), 
                                 current_path=folder_path)
        elif os.path.isfile(full_path):
            try:
                with open(full_path, 'r', encoding='utf-8') as f:
                    content = f.read()
            except UnicodeDecodeError:
                with open(full_path, 'r', encoding='gbk') as f:
                    content = f.read()
            return render_template("c2_log.html", file_name=folder_path, file_content=content)
        else:
            return render_template("error.html", error="文件或目录不存在")
    except Exception as e:
        return render_template("error.html", error=f"访问文件出错: {str(e)}")




@app.route("/params")
def params_list():
    params = fleet.list_params()
    return render_template("params.html", 
                         params=params,
                         get_param_description=fleet.get_param_description)

@app.route("/params/<param_name>", methods=["GET", "POST"])
def param_detail(param_name):
    if request.method == "POST":
        value = request.form.get("value")
        if not fleet.validate_param(param_name, value):
            return render_template("error.html", error="参数值格式不正确")
        if fleet.write_param(param_name, value):
            return redirect(url_for("params_list"))
        else:
            return render_template("error.html", error="参数写入失败")

    value = fleet.read_param(param_name)
    description = fleet.get_param_description(param_name)
    return render_template("param_detail.html", 
                         param_name=param_name, 
                         param_value=value,
                         param_description=description)


# 假设截图保存在 /data/media/0/ui_screenshots 目录
SCREENSHOT_DIR = "/data/media/0/ui_screenshots" # 确保这里也是 /data/media/0/ui_screenshots
UI_TOUCH_SOCKET = '/tmp/ui_touch_socket' # UI touch socket path

def wait_for_wifi(interface="wlan0", timeout=30, delay=2):
    print("Waiting for Wi-Fi connection...")
    for _ in range(timeout // delay):
        try:
            # Note: 'ip route show dev wlan0' is a Linux command.
            # On Windows, you might need a different approach to check network connectivity.
            # For a cross-platform solution or if running on the target device (likely Linux),
            # this might work, but for Windows development environment, it might fail.
            # Assuming target environment is Linux where this code runs.
            result = subprocess.run(["ip", "route", "show", "dev", interface],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            if result.stdout.strip():
                print("Wi-Fi is up.")
                return True
        except Exception:
            pass
        time.sleep(delay)
    print("Wi-Fi not detected after timeout.")
    return False

def clean_old_screenshots():
    print("Cleaning old screenshots...")
    files = glob.glob(os.path.join(SCREENSHOT_DIR, "ui_frame_*.png"))

    # 根据文件名中的时间戳排序文件
    files.sort(key=lambda x: os.path.getmtime(x)) # 或者根据文件名解析时间戳

    # 保留最新的 100 个文件，删除其余的
    files_to_delete = files[:-100] if len(files) > 100 else []

    deleted_count = 0
    for f in files_to_delete:
        try:
            os.remove(f)
            deleted_count += 1
        except Exception as e:
            print(f"Failed to delete {f}: {e}")
    print(f"Deleted {deleted_count} old screenshots.")
    print(f"Remaining screenshots: {len(files) - deleted_count}")


def periodic_cleanup(interval_seconds=120): # 每120秒（2分钟）清理一次
    while True:
        time.sleep(interval_seconds)
        clean_old_screenshots()
        print(f"Periodic cleanup finished. Next cleanup in {interval_seconds} seconds.")


def send_input_to_ui(event_data):
    """Send input events to Qt application via Unix socket"""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(UI_TOUCH_SOCKET)
            message = json.dumps({
                **event_data,
                'timestamp': time.time()
            })
            sock.send(message.encode())
            return True
    except Exception as e:
        print(f"Failed to send input event: {e}")
        return False

@app.route('/screenshot_live') # Renamed from '/'
def screenshot_live_page():
    params = Params()
    if not params.get_bool("dp_screenshotserver"):
        return render_template("error.html", error="Screenshot server is disabled."), 403
    # Assuming you save the HTML content to templates/screenshot_live.html
    return render_template('screenshot_live.html', timestamp='init')

@app.route('/screenshot_live/screenshot') # Added prefix to avoid conflict
def latest_screenshot():
    params = Params()
    if not params.get_bool("dp_screenshotserver"):
        return "Screenshot server is disabled.", 403
    files = sorted(glob.glob(os.path.join(SCREENSHOT_DIR, "ui_frame_*.png")))
    if not files:
        return "No screenshot found", 404
    # Use send_from_directory to serve the file securely
    return send_from_directory(SCREENSHOT_DIR, os.path.basename(files[-1]), mimetype='image/png')


@app.route('/screenshot_live/input', methods=['POST']) # Added prefix
def handle_input():
    params = Params()
    if not params.get_bool("dp_screenshotserver"):
        return jsonify({'status': 'error', 'message': 'Screenshot server is disabled.'}), 403

    data = request.get_json()
    event_type = data.get('type', 'click')

    # 新增：校验 img_w/img_h
    img_w = data.get('img_w')
    img_h = data.get('img_h')
    if img_w is None or img_h is None:
        return jsonify({'status': 'error', 'message': 'img_w/img_h required'}), 400

    print(f"Received {event_type} event: {data}")

    if send_input_to_ui(data):
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to send input event'}), 500

# Backward compatibility with old touch endpoint (optional, added prefix)
@app.route('/screenshot_live/touch', methods=['POST'])
def handle_touch():
    params = Params()
    if not params.get_bool("dp_screenshotserver"):
        return jsonify({'status': 'error', 'message': 'Screenshot server is disabled.'}), 403
    data = request.get_json()
    data['type'] = 'click'  # Convert old touch events to click events
    return handle_input()

# --- End Screenshot Server Functions and Routes ---


def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("fleet_manager: failed to set core affinity")
  app.secret_key = secrets.token_hex(32)
  #schedule_pin_generate()

  # --- Start Screenshot Server Background Tasks ---
  params = Params()
  if params.get_bool("dp_screenshotserver"):
      # wait_for_wifi() # Optional, depending on environment
      clean_old_screenshots() # 启动时清理一次

      # 启动定时清理线程，每120秒（2分钟）清理一次
      cleanup_thread = threading.Thread(target=periodic_cleanup, args=(120,), daemon=True)
      cleanup_thread.start()
      print("Periodic cleanup thread started with 120s interval.")
  else:
      print("Screenshot server is disabled. Background tasks not started.")
  # --- End Screenshot Server Background Tasks ---

  app.run(host="0.0.0.0", port=5050) # Fleet manager runs on port 5050


if __name__ == '__main__':
  main()
