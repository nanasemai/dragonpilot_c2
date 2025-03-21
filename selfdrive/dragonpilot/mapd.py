#!/usr/bin/env python3
#pylint: skip-file
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

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, set_core_affinity, set_realtime_priority
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
import json
from cereal import log
from pathlib import Path
import overpy
import os
import subprocess
from openpilot.common.conversions import Conversions as CV
import re
import time

OSM_QUERY = ["/data/media/0/osm/bin/osm3s_query", "--db-dir=/data/media/0/osm/db/"]
OSM_LOG_PATH = '/data/media/0/c2_logs/osm_logs/'

OSM_ONLINE_QUERY_THRESHOLD = 5 # secs
OSM_LOCAL_QUERY_THRESHOLD = 3 # times

class OSM():
  def __init__(self, last_gps_pos):
    self._api = overpy.Overpass()

    self._local_osm_query_fail_count = 0
    self._last_gps_pos = last_gps_pos
    self._fetch_time_prev = 0.
    self._way_tags = []

    self.way_id = 0
    self.road_name = None
    self.speed_limit = 0
    # 修改日志初始化
    cloudlog.bind_global(module='mapd')
    cloudlog.bind(component='osm')
    # 确保日志目录存在
    Path(OSM_LOG_PATH).mkdir(parents=True, exist_ok=True)

    if os.path.isdir("/data/media/0/osm/bin/") and os.path.isdir("/data/media/0/osm/db/"):
      self.local_osm_enabled = True
      cloudlog.info("OSM状态", local=True, db_path="/data/media/0/osm/db/")
    else:
      self.local_osm_enabled = False
      cloudlog.info("OSM状态", local=False)
    #cloudlog.info(f"本地OSM状态: {'已启用' if self.local_osm_enabled else '未启用'}")

  def _process_res(self, res):
    if len(res) > 0:
      self._way_tags = res[0].tags
      self.way_id = res[0].id

  def fetch_tags_around_location(self, latitude, longitude):
    if len(self._way_tags) > 0 and len(self._last_gps_pos) > 0 and self._last_gps_pos["latitude"] == latitude and self._last_gps_pos["longitude"] == longitude:
      return self._way_tags
    q = f"""
    way(around:10, {latitude}, {longitude})
          [highway]
          [highway~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential)$"];
    (
        way._["name"];
        way._["maxspeed"];
        (._;>;);
    );
    out tags;
    """
    if self.local_osm_enabled:
      try:
        completion = subprocess.run(OSM_QUERY + [f"--request={q}"], check=True, capture_output=True)
        res = self._api.parse_xml(completion.stdout).ways
        self._process_res(res)
      except Exception as e:
        self._local_osm_query_fail_count += 1
        cloudlog.exception(f"本地OSM查询失败: {str(e)}")

    # use remote OSM when local osm is not enabled or failed too many times
    if not self.local_osm_enabled or self._local_osm_query_fail_count >= OSM_LOCAL_QUERY_THRESHOLD:
      fetch_time = time.monotonic()
      if fetch_time - self._fetch_time_prev < OSM_ONLINE_QUERY_THRESHOLD:
        return
      try:
        res = self._api.query(q).ways
        self._process_res(res)
        self._local_osm_query_fail_count = 0
        cloudlog.debug("在线OSM查询成功", way_count=len(res))
      except Exception as e:
        cloudlog.error("在线OSM查询失败", error=str(e), exc=True)
      self._fetch_time_prev = fetch_time

    self._process_tags()

  def _process_tags(self):
    if len(self._way_tags) > 0:
      self._process_road_name_tag()
      self._process_speed_limit_tags()
      cloudlog.debug(f"道路信息: 名称={self.road_name}, 限速={self.speed_limit}")

  def _process_road_name_tag(self):
    tag_value = self._way_tags.get("name")
    if tag_value is not None:
      self.road_name = str(tag_value).strip()
    else:
      self.road_name = ""

  def _process_speed_limit_tags(self):
    self.speed_limit = self._speed_limit_for_osm_tag_value(self._way_tags.get("maxspeed"))

  def _speed_limit_for_osm_tag_value(self, value):
    if value is None:
      return 0.

    limit = self._speed_limit_value_for_value(value)
    if limit is not None:
      return limit

    v = re.match(r'^\s*([A-Z]{2}):([a-z_]+):?([0-9]{1,3})?(\s+)?(mph)?\s*', value)
    if v is None:
      return 0.

    if v[2] == "zone" and v[3] is not None:
      conv = CV.MPH_TO_MS if v[5] is not None and v[5] == "mph" else CV.KPH_TO_MS
      limit = conv * float(v[3])

    return limit if limit is not None else 0.

  def _speed_limit_value_for_value(self, value):
    v = re.match(r'^\s*([0-9]{1,3})\s*?(mph)?\s*$', value)
    if v is None:
      return None
    conv = CV.MPH_TO_MS if v[2] is not None and v[2] == "mph" else CV.KPH_TO_MS
    return conv * float(v[1])


class MapD():
  def __init__(self, position_service):
    last_gps_params = Params().get('LastGPSPosition')
    self.last_gps_pos = json.loads(last_gps_params) if last_gps_params is not None else []
    self.osm = OSM(self.last_gps_pos)

    self.position_service = position_service
    self._pause_query = False
    self._longitude = None
    self._latitude = None

    self._way_id_prev = 0
    self._road_name_prev = None
    self._same_road_count = 0

  def apply_last_gps_pos(self):
    if self.last_gps_pos is not None and len(self.last_gps_pos) > 0:
      self._latitude = self.last_gps_pos["latitude"]
      self._longitude = self.last_gps_pos["longitude"]
      cloudlog.info(f"使用上次GPS位置: {self._latitude}, {self._longitude}")

  def update_car_state(self, sm):
    sock = 'carState'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    car_state = sm[sock]
    self._pause_query = car_state.vEgo <= 2.78 # 10km/h

  def update_position(self, sm):
    if self.position_service == "liveLocationKalman":
      self.update_locationd(sm)
    else:
      self.update_gps(sm)

  def update_locationd(self, sm):
    sock = 'liveLocationKalman'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    location = sm[sock]
    location_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if not location_valid:
      return

    self._latitude = location.positionGeodetic.value[0]
    self._longitude = location.positionGeodetic.value[1]
    cloudlog.debug(f"使用定位服务位置: {self._latitude}, {self._longitude}")
    cloudlog.debug("位置更新",
                  source="locationd",
                  valid=location_valid,
                  lat=self._latitude,
                  lon=self._longitude)
  def update_gps(self, sm):
    sock = 'gpsLocationExternal'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    log = sm[sock]
    self.last_gps = log

    if log.flags % 2 == 0:
      return

    self._latitude = log.latitude
    self._longitude = log.longitude
    cloudlog.debug(f"使用GPS位置: {self._latitude}, {self._longitude}")
    cloudlog.debug("位置更新",
                  source="gps",
                  valid=log.flags % 2 == 1,
                  lat=self._latitude,
                  lon=self._longitude)

def mapd_thread(sm=None, pm=None):
  use_locationd = False
  position_service = "liveLocationKalman" if use_locationd else "gpsLocationExternal"

  set_core_affinity([1,])
  set_realtime_priority(1)
  mapd = MapD(position_service)
  rk = Ratekeeper(0.5, print_delay_threshold=None)

  if sm is None:
    sm = messaging.SubMaster(["carState", position_service])
  if pm is None:
    pm = messaging.PubMaster(['liveMapData'])

  cloudlog.info("地图服务启动",
                 position_service=position_service,
                 use_locationd=use_locationd)
  mapd.apply_last_gps_pos()

  while True:
    try:
      sm.update()
      mapd.update_car_state(sm)
      mapd.update_position(sm)
      mapd.fetch_osm_data()
      mapd.publish(pm)
      rk.keep_time()
    except Exception as e:
      cloudlog.exception("地图服务异常", error=str(e))

def main(sm=None, pm=None):
  mapd_thread(sm, pm)

if __name__ == "__main__":
  main()
