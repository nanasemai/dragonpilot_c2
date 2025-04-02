#!/usr/bin/env python3
import os
import random
import secrets
import threading
import time
from flask import Flask, render_template, Response, request, send_from_directory, session, redirect, url_for
from openpilot.common.realtime import set_core_affinity
import openpilot.system.fleetmanager.helpers as fleet
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

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
        with open(fleet.ERROR_LOGS_PATH + file_name, 'r', encoding='gbk') as f:  # 修正路径
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


def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("fleet_manager: failed to set core affinity")
  app.secret_key = secrets.token_hex(32)
  #schedule_pin_generate()
  app.run(host="0.0.0.0", port=5050)


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


if __name__ == '__main__':
  main()
