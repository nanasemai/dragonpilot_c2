#!/usr/bin/env python3
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
import time
from pathlib import Path
from openpilot.common.params import Params
from openpilot.system.version import get_version, get_branch
from openpilot.common.swaglog import cloudlog
# from openpilot.common.realtime import set_core_affinity, set_realtime_priority

# for uploader
from openpilot.selfdrive.loggerd.xattr_cache import getxattr, setxattr
import glob
import requests
# import json

# customisable values
GPX_LOG_PATH = '/data/media/0/gpx_logs/'
LOG_HERTZ = 1/10 # 0.1 Hz = 10 sec, higher for higher accuracy, 10hz seems fine

# uploader
UPLOAD_ATTR_NAME = 'user.upload'
UPLOAD_ATTR_VALUE = b'1'

# osm api
API_HEADER = {'Authorization': 'Bearer 2pvUyXfk9vizuh7PwQFSEYBtFWcM-Pu7vxApUjSA0fc'}
VERSION_URL = 'https://api.openstreetmap.org/api/versions'
UPLOAD_URL = 'https://api.openstreetmap.org/api/0.6/gpx/create'

_DEBUG = False

def _debug(msg):
  if not _DEBUG:
    return
  print(msg, flush=True)

class GpxUploader():
  def __init__(self):
    self._delete_after_upload = True
    self._version = get_version()
    self._branch = get_branch()
    cloudlog.debug("初始化 GpxUploader")
    # 确保日志和视频目录存在
    Path(GPX_LOG_PATH).mkdir(parents=True, exist_ok=True)

  def _identify_vehicle(self):
    cloudlog.debug(f"GpxUploader初始化 - _delete_after_upload = {self._delete_after_upload}")

  def _is_online(self):
    try:
      r = requests.get(VERSION_URL, headers=API_HEADER)
      cloudlog.debug(f"检查在线状态: {r.status_code}")
      return r.status_code >= 200
    except Exception as e:
      cloudlog.error(f"检查在线状态失败: {str(e)}")
      return False

  def _get_is_uploaded(self, filename):
    result = getxattr(filename, UPLOAD_ATTR_NAME) is not None
    cloudlog.debug(f"文件 {filename} 上传状态: {result}")
    return result

  def _set_is_uploaded(self, filename):
    cloudlog.debug(f"标记文件已上传: {filename}")
    setxattr(filename, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)

  def _do_upload(self, filename):
    fn = os.path.basename(filename)
    data = {
      'description': f"Routes from dragonpilot {self._branch} / {self._version}.",
      'visibility': 'identifiable'
    }
    files = {
      "file": (fn, open(filename, 'rb'))
    }
    try:
      r = requests.post(UPLOAD_URL, files=files, data=data, headers=API_HEADER)
      cloudlog.debug(f"上传文件 {filename} - 状态码: {r.status_code}")
      return r.status_code == 200
    except Exception as e:
      cloudlog.error(f"上传文件失败 {filename}: {str(e)}")
      return False

  def run(self):
    time.sleep(10)
    self._identify_vehicle()
    while True:
      is_offroad = Params().get_bool("IsOffroad")
      files = self._get_files_to_be_uploaded()
      if len(files) == 0:
        if is_offroad and self._delete_after_upload:
          for file in self._get_files():
            os.remove(file)
            cloudlog.info(f"清理文件: {file}")
        cloudlog.debug("无待上传文件")
      elif not self._is_online() and self._delete_after_upload:
        cloudlog.info("离线状态，删除待上传文件")
        for file in files:
          os.remove(file)
          cloudlog.info(f"删除文件: {file}")
      else:
        for file in files:
          if self._do_upload(file):
            if self._delete_after_upload:
              cloudlog.info(f"上传成功，删除文件: {file}")
              os.remove(file)
            else:
              cloudlog.info(f"上传成功，标记文件: {file}")
              self._set_is_uploaded(file)
      time.sleep(60)

def gpx_uploader_thread():
  gpx_uploader = GpxUploader()
  gpx_uploader.run()

def main():
  gpx_uploader_thread()

if __name__ == "__main__":
  main()
