#!/usr/bin/env python3
import os
import subprocess
import time
import datetime
import requests
from panda import Panda
from openpilot.common.params import Params
from openpilot.common.time import MIN_DATE
import cereal.messaging as messaging

PARAMS = Params()
LAST_TIME_KEY = "LastValidTime"
NTP_SERVERS = [
  "ntp.aliyun.com",         # 阿里云
  "ntp.tencent.com",        # 腾讯
  "cn.ntp.org.cn",          # 中国 NTP 池
  "ntp.ntsc.ac.cn"          # 中国科学院国家授时中心
]

def save_last_valid_time(time_value, logger):
  if time_value and time_value > MIN_DATE:
    try:
      timestamp = int(time_value.timestamp())
      if timestamp > 0:
        PARAMS.put(LAST_TIME_KEY, str(timestamp))
        logger.debug(f"已保存有效时间戳: {timestamp} ({time_value.strftime('%Y-%m-%d %H:%M:%S')})")
    except (ValueError, TypeError, OSError) as e:
      logger.error(f"保存时间戳失败: {str(e)}")

def get_ntp_time(logger):
  logger.info(f"开始尝试从 {len(NTP_SERVERS)} 个NTP服务器获取时间...")
  for server in NTP_SERVERS:
    try:
      logger.debug(f"正在连接NTP服务器: {server}")
      response = requests.get(f"https://{server}", timeout=2, verify=False)
      if response.ok and 'date' in response.headers:
        server_time = datetime.datetime.strptime(response.headers['date'], '%a, %d %b %Y %H:%M:%S %Z')
        logger.info(f"NTP服务器 {server} 同步成功: {server_time.strftime('%Y-%m-%d %H:%M:%S')}")
        return server_time
    except Exception as e:
      logger.debug(f"NTP服务器 {server} 同步失败: {str(e)}")
  logger.warning("所有NTP服务器同步失败")
  return None

def get_gps_time(logger):
  try:
    logger.info("正在尝试获取GPS时间...")
    sm = messaging.SubMaster(['gpsLocationExternal'])
    # 等待最多 5 秒获取 GPS 数据
    for i in range(50):
      sm.update()
      if sm.updated['gpsLocationExternal'] and sm.valid['gpsLocationExternal']:
        log = sm['gpsLocationExternal']
        # 检查 GPS fix 是否有效
        if log.flags % 2 != 0:
          gps_time = datetime.datetime.fromtimestamp(log.unixTimestampMillis / 1000.0)
          if gps_time and gps_time > MIN_DATE:
            logger.info(f"GPS时间获取成功: {gps_time.strftime('%Y-%m-%d %H:%M:%S')}")
            return gps_time
      if i % 10 == 0:  # 每10次循环记录一次等待信息
        logger.debug(f"等待GPS数据... ({i/10}/5秒)")
      time.sleep(0.1)
    logger.warning("等待GPS数据超时")
  except Exception as e:
    logger.error(f"GPS时间同步失败: {str(e)}")
  return None

def get_last_valid_time(logger):
  try:
    logger.info("正在获取上次保存的有效时间...")
    timestamp = PARAMS.get(LAST_TIME_KEY)
    if timestamp:
      time_value = datetime.datetime.fromtimestamp(int(timestamp))
      logger.info(f"找到上次有效时间: {time_value.strftime('%Y-%m-%d %H:%M:%S')}")
      return time_value
    logger.debug("未找到上次有效时间记录")
  except Exception as e:
    logger.error(f"获取上次有效时间失败: {str(e)}")
  return None

def set_system_time(time_value, source, logger):
  try:
    if time_value and time_value > MIN_DATE:
      time_str = time_value.strftime('%Y-%m-%d %H:%M:%S')
      logger.info(f"正在设置系统时间为 {time_str} (来源: {source})...")
      result = os.system(f"TZ=UTC date -s '{time_str}'") == 0
      if result:
        logger.info(f"✓ 系统时间设置成功: {time_str} (来源: {source})")
        save_last_valid_time(time_value, logger)
        return True
      else:
        logger.error(f"✗ 设置系统时间失败 (来源: {source})")
    else:
      logger.warning(f"无效的时间值 (来源: {source}): {time_value}")
    return False
  except Exception as e:
    logger.error(f"设置系统时间出错 (来源: {source}): {str(e)}")
    return False

def set_time(logger):
  logger.info("=== 开始时间同步流程 ===")
  sys_time = datetime.datetime.today()
  if sys_time > MIN_DATE:
    logger.info(f"系统时间已有效: {sys_time.strftime('%Y-%m-%d %H:%M:%S')}")
    save_last_valid_time(sys_time, logger)
    return

  logger.warning(f"系统时间无效: {sys_time.strftime('%Y-%m-%d %H:%M:%S')}, 开始尝试同步...")

  # 1. 尝试网络时间同步
  logger.info("【方法1】尝试网络时间同步...")
  ntp_time = get_ntp_time(logger)
  if ntp_time and set_system_time(ntp_time, "NTP", logger):
    logger.info("=== 时间同步完成 ===")
    return

  # 2. 尝试使用上次保存的时间
  logger.info("【方法2】尝试使用上次保存的有效时间...")
  last_time = get_last_valid_time(logger)
  if last_time and set_system_time(last_time, "上次有效时间", logger):
    logger.info("=== 时间同步完成 ===")
    return

  # 3. 尝试GPS时间（检查 ubloxd 进程是否运行）
  logger.info("【方法3】尝试使用GPS时间...")
  try:
    if os.path.exists("/proc/1"):  # 检查是否在 Linux 系统
      ubloxd_running = subprocess.run(["pgrep", "ubloxd"], capture_output=True).returncode == 0
      if ubloxd_running:
        logger.info("ubloxd进程正在运行，尝试获取GPS时间...")
        gps_time = get_gps_time(logger)
        if gps_time and set_system_time(gps_time, "GPS", logger):
          logger.info("=== 时间同步完成 ===")
          return
      else:
        logger.warning("ubloxd进程未运行，无法获取GPS时间")
  except Exception as e:
    logger.error(f"检查ubloxd状态失败: {str(e)}")

  # 4. 尝试Panda RTC时间
  logger.info("【方法4】尝试使用Panda RTC时间...")
  try:
    ps = Panda.list()
    if len(ps) > 0:
      logger.info(f"找到 {len(ps)} 个Panda设备")
      for i, s in enumerate(ps):
        logger.debug(f"正在连接Panda设备 {i+1}/{len(ps)}: {s}")
        with Panda(serial=s) as p:
          if not p.is_internal():
            logger.debug(f"设备 {s} 不是内部Panda，跳过")
            continue

          logger.info(f"正在从内部Panda设备 {s} 获取时间...")
          panda_time = p.get_datetime()
          if panda_time:
            logger.info(f"Panda RTC时间: {panda_time.strftime('%Y-%m-%d %H:%M:%S')}")
            if set_system_time(panda_time, "Panda RTC", logger):
              logger.info("=== 时间同步完成 ===")
              return
          else:
            logger.warning(f"Panda设备 {s} 未返回有效时间")
    else:
      logger.warning("未找到Panda设备")
  except Exception as e:
    logger.error(f"获取Panda时间失败: {str(e)}")

  logger.error("❌ 所有时间源都无法获取有效时间，时间同步失败")
  logger.info("=== 时间同步流程结束 ===")

if __name__ == "__main__":
  import logging
  logging.basicConfig(level=logging.INFO, 
                     format='%(asctime)s - %(levelname)s - %(message)s',
                     datefmt='%Y-%m-%d %H:%M:%S')
  logger = logging.getLogger("时间同步")
  set_time(logger)