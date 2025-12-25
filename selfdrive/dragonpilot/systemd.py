import time
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.dragonpilot.dashcamd import Dashcamd
from openpilot.common.realtime import Ratekeeper
from pathlib import Path

HERTZ = 1

def dashcam_thread():
  # 修改订阅消息，添加 carState
  sm = messaging.SubMaster(['deviceState', 'carState'])
  params = Params()
  frame = 0

  # 添加 P 档计时相关变量
  last_park_time = 0
  park_detected = False
  PARK_RESTART_DELAY = 5  # 5秒后重新开始录制

  # 读取配置
  dashcam_config = {
    'enabled': params.get_bool("dp_on_road_dashcam"),
    'quality': int(params.get("dp_dashcam_quality", encoding='utf8') or "1"), # 0=低 1=中 2=高
    'duration': int(params.get("dp_dashcam_duration", encoding='utf8') or "180"),
    'kept_hours': int(params.get("dp_dashcam_kept_hours", encoding='utf8') or "15")
  }

  # 初始化行车记录仪
  dashcam = Dashcamd(dashcam_config)
  #cloudlog.debug(f"初始化配置: {dashcam_config}")

  # 检查是否处于开发者模式
  dev_mode = params.get_bool("DPDEVMODE")

  # 默认的空闲空间百分比
  free_space = 100.0

  # 如果启用了行车记录仪功能或处于开发者模式，立即开始录制
  started = dashcam_config['enabled'] or dev_mode
  if started:
    #cloudlog.info(f"行车记录仪启动: enabled={dashcam_config['enabled']}, dev_mode={dev_mode}"H)
    dashcam.run(started=True, free_space=free_space)

  rk = Ratekeeper(HERTZ, print_delay_threshold=None)

  while True:
    if frame % (HERTZ * 3) == 0:
      sm.update(0)

      # 检测车辆是否挂 P 档
      if sm.updated['carState']:
        is_park = sm['carState'].gearShifter == 'park'

        if is_park and not park_detected:
          park_detected = True
          last_park_time = time.monotonic()
        elif not is_park:
          park_detected = False
          last_park_time = 0

        # 如果持续挂 P 档超过设定时间，重新开始录制
        if park_detected and (time.monotonic() - last_park_time) >= PARK_RESTART_DELAY:
          if started:
            #cloudlog.info("检测到持续停车，重新开始录制"H)
            dashcam.restart_recording()
          last_park_time = time.monotonic()  # 重置计时器

      # 无条件获取和更新可用空间
      if sm.updated['deviceState']:
          new_free_space = sm['deviceState'].freeSpacePercent
          # 只记录空闲空间显著变化
          if free_space is None or abs(new_free_space - free_space) > 0.5:  # 变化超过0.5%才记录
            cloudlog.info(f"系统空闲空间: {new_free_space}%")
            free_space = new_free_space
      
      # 当空间不足时，直接调用清理方法进行紧急清理
      if free_space is not None and free_space < 15.0:  # 当空间低于15%时立即清理
          cloudlog.warning(f"警告：空间不足({free_space}%), 强制触发清理")
          dashcam._clean_up_space()

      # 检查配置变更
      new_enabled = params.get_bool("dp_on_road_dashcam")
      new_quality = int(params.get("dp_dashcam_quality", encoding='utf8') or "1")  # 0=低 1=中 2=高
      new_duration = int(params.get("dp_dashcam_duration", encoding='utf8') or "180")
      new_kept_hours = int(params.get("dp_dashcam_kept_hours", encoding='utf8') or "15")
      new_dev_mode = params.get_bool("DPDEVMODE")

      config_changed = (
        new_enabled != dashcam_config['enabled'] or
        new_quality != dashcam_config['quality'] or
        new_duration != dashcam_config['duration'] or
        new_kept_hours != dashcam_config['kept_hours'] or
        new_dev_mode != dev_mode
      )

      if config_changed:
        old_config = dashcam_config.copy()
        dashcam_config.update({
          'enabled': new_enabled,
          'quality': new_quality,
          'duration': new_duration,
          'kept_hours': new_kept_hours
        })
        dev_mode = new_dev_mode

        #cloudlog.info(f"配置变更: old_config={old_config}, new_config={dashcam_config}, dev_mode={dev_mode}"H)
        dashcam.update_config(dashcam_config)

      # 根据配置和开发者模式决定是否启动录制
      new_started = dashcam_config['enabled'] or dev_mode
      if new_started != started:
        started = new_started
        #cloudlog.info(f"录制状态变更: started={started}"H)

      # 运行行车记录仪
      dashcam.run(started=started, free_space=free_space)

    frame += 1
    rk.keep_time()

def main():
  try:
    dashcam_thread()
  except Exception as e:
    cloudlog.exception(f"行车记录仪服务异常退出: {str(e)}")

if __name__ == "__main__":
  main()
