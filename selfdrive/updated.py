#!/usr/bin/env python3
import os
import re
import datetime
import subprocess
import psutil
import shutil
import signal
import fcntl
import time
import threading
from collections import defaultdict
from pathlib import Path
from typing import List, Union, Optional
from markdown_it import MarkdownIt

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time import system_time_valid
from openpilot.system.hardware import AGNOS, HARDWARE, EON
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.alertmanager import set_offroad_alert
from openpilot.system.version import is_tested_branch
from openpilot.system.hardware.eon.neos import download_neos_update

LOCK_FILE = os.getenv("UPDATER_LOCK_FILE", "/tmp/safe_staging_overlay.lock")
STAGING_ROOT = os.getenv("UPDATER_STAGING_ROOT", "/data/safe_staging")

NEOSUPDATE_DIR = os.getenv("UPDATER_NEOSUPDATE_DIR", "/data/neoupdate")

OVERLAY_UPPER = os.path.join(STAGING_ROOT, "upper")
OVERLAY_METADATA = os.path.join(STAGING_ROOT, "metadata")
OVERLAY_MERGED = os.path.join(STAGING_ROOT, "merged")
FINALIZED = os.path.join(STAGING_ROOT, "finalized")

OVERLAY_INIT = Path(os.path.join(BASEDIR, ".overlay_init"))

DAYS_NO_CONNECTIVITY_MAX = 14     # do not allow to engage after this many days
DAYS_NO_CONNECTIVITY_PROMPT = 10  # send an offroad prompt after this many days

class UserRequest:
  NONE = 0
  CHECK = 1
  FETCH = 2

class WaitTimeHelper:
  def __init__(self):
    self.ready_event = threading.Event()
    self.user_request = UserRequest.NONE
    signal.signal(signal.SIGHUP, self.update_now)
    signal.signal(signal.SIGUSR1, self.check_now)

  def update_now(self, signum: int, frame) -> None:
    cloudlog.info("caught SIGHUP, attempting to downloading update")
    self.user_request = UserRequest.FETCH
    self.ready_event.set()

  def check_now(self, signum: int, frame) -> None:
    cloudlog.info("caught SIGUSR1, checking for updates")
    self.user_request = UserRequest.CHECK
    self.ready_event.set()

  def sleep(self, t: float) -> None:
    self.ready_event.wait(timeout=t)

def write_time_to_param(params, param) -> None:
  t = datetime.datetime.utcnow()
  params.put(param, t.isoformat().encode('utf8'))

def read_time_from_param(params, param) -> datetime.datetime:
  t = params.get(param, encoding='utf8')
  try:
    return datetime.datetime.fromisoformat(t)
  except (TypeError, ValueError):
    pass
  return None

def run(cmd: List[str], cwd: Optional[str] = None) -> str:
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding='utf8')


def set_consistent_flag(consistent: bool) -> None:
  os.sync()
  consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
  if consistent:
    consistent_file.touch()
  elif not consistent:
    consistent_file.unlink(missing_ok=True)
  os.sync()

def parse_release_notes(basedir: str) -> bytes:
  try:
    with open(os.path.join(basedir, "CHANGELOGS.md"), "rb") as f:
      r = f.read().split(b'\n\n', 1)[0]  # Slice latest release notes
    try:
      return bytes(MarkdownIt().render(r.decode("utf-8")), encoding="utf-8")
    except Exception:
      return r + b"\n"
  except FileNotFoundError:
    pass
  except Exception:
    cloudlog.exception("failed to parse release notes")
  return b""

def setup_git_options(cwd: str) -> None:
    # 项目级别配置
    git_local_cfg = [
        ("core.trustctime", "false"),
        ("core.checkStat", "minimal"),
        ("gc.auto", "0"),
        ("gc.autoDetach", "false"),
    ]
    
    # 全局配置
    git_global_cfg = [
        ("protocol.version", "2"),
        ("http.postBuffer", "1048576000"),
        ("http.version", "HTTP/1.1"),
        ("http.lowSpeedLimit", "0"),
        ("http.lowSpeedTime", "999999"),
        ("http.sslVerify", "false"),
    ]

    # 设置项目级别配置
    for option, value in git_local_cfg:
        run(["git", "config", option, value], cwd)

    # 设置全局配置
    for option, value in git_global_cfg:
        run(["git", "config", "--global", option, value], cwd)


def dismount_overlay() -> None:
  if os.path.ismount(OVERLAY_MERGED):
    cloudlog.info("unmounting existing overlay")
    args = ["umount", "-l", OVERLAY_MERGED]
    if AGNOS:
      args = ["sudo"] + args
    run(args)


def init_overlay() -> None:

  # Re-create the overlay if BASEDIR/.git has changed since we created the overlay
  if OVERLAY_INIT.is_file() and os.path.ismount(OVERLAY_MERGED):
    git_dir_path = os.path.join(BASEDIR, ".git")
    new_files = run(["find", git_dir_path, "-newer", str(OVERLAY_INIT)])
    if not len(new_files.splitlines()):
      # A valid overlay already exists
      return
    else:
      cloudlog.info(".git directory changed, recreating overlay")

  cloudlog.info("preparing new safe staging area")

  params = Params()
  params.put_bool("UpdateAvailable", False)
  set_consistent_flag(False)
  dismount_overlay()
  args = ["rm", "-rf", STAGING_ROOT]
  if AGNOS:
    args = ["sudo"] + args
    run(args)
  if os.path.isdir(STAGING_ROOT):
    shutil.rmtree(STAGING_ROOT)

  for dirname in [STAGING_ROOT, OVERLAY_UPPER, OVERLAY_METADATA, OVERLAY_MERGED]:
    os.mkdir(dirname, 0o755)

  if os.lstat(BASEDIR).st_dev != os.lstat(OVERLAY_MERGED).st_dev:
    raise RuntimeError("base and overlay merge directories are on different filesystems; not valid for overlay FS!")

  # Leave a timestamped canary in BASEDIR to check at startup. The device clock
  # should be correct by the time we get here. If the init file disappears, or
  # critical mtimes in BASEDIR are newer than .overlay_init, continue.sh can
  # assume that BASEDIR has used for local development or otherwise modified,
  # and skips the update activation attempt.
  consistent_file = Path(os.path.join(BASEDIR, ".overlay_consistent"))
  if consistent_file.is_file():
    consistent_file.unlink()
  OVERLAY_INIT.touch()

  os.sync()
  overlay_opts = f"lowerdir={BASEDIR},upperdir={OVERLAY_UPPER},workdir={OVERLAY_METADATA}"

  mount_cmd = ["mount", "-t", "overlay", "-o", overlay_opts, "none", OVERLAY_MERGED]
  if AGNOS:
    run(["sudo"] + mount_cmd)
    run(["sudo", "chmod", "755", os.path.join(OVERLAY_METADATA, "work")])
  else:
    run(mount_cmd)

  git_diff = run(["git", "diff"], OVERLAY_MERGED)
  params.put("GitDiff", git_diff)
  cloudlog.info(f"git diff output:\n{git_diff}")


def finalize_update() -> None:
  """Take the current OverlayFS merged view and finalize a copy outside of
  OverlayFS, ready to be swapped-in at BASEDIR. Copy using shutil.copytree"""

  # Remove the update ready flag and any old updates
  cloudlog.info("creating finalized version of the overlay")
  set_consistent_flag(False)

  # Copy the merged overlay view and set the update ready flag
  if os.path.exists(FINALIZED):
    shutil.rmtree(FINALIZED)
  shutil.copytree(OVERLAY_MERGED, FINALIZED, symlinks=True)

  run(["git", "reset", "--hard"], FINALIZED)
  run(["git", "submodule", "foreach", "--recursive", "git", "reset", "--hard"], FINALIZED)

  cloudlog.info("Starting git cleanup in finalized update")
  t = time.monotonic()
  try:
    run(["git", "gc"], FINALIZED)
    run(["git", "lfs", "prune"], FINALIZED)
    cloudlog.event("Done git cleanup", duration=time.monotonic() - t)
  except subprocess.CalledProcessError:
    cloudlog.exception(f"Failed git cleanup, took {time.monotonic() - t:.3f} s")

  set_consistent_flag(True)
  cloudlog.info("done finalizing overlay")


def handle_agnos_update() -> None:
  from openpilot.system.hardware.tici.agnos import flash_agnos_update, get_target_slot_number

  cur_version = HARDWARE.get_os_version()
  updated_version = run(["bash", "-c", r"unset AGNOS_VERSION && source launch_env.sh && \
                          echo -n $AGNOS_VERSION"], OVERLAY_MERGED).strip()

  cloudlog.info(f"AGNOS version check: {cur_version} vs {updated_version}")
  if cur_version == updated_version:
    return

  # prevent an openpilot getting swapped in with a mismatched or partially downloaded agnos
  set_consistent_flag(False)

  cloudlog.info(f"Beginning background installation for AGNOS {updated_version}")
  set_offroad_alert("Offroad_NeosUpdate", True)

  manifest_path = os.path.join(OVERLAY_MERGED, "system/hardware/tici/agnos.json")
  target_slot_number = get_target_slot_number()
  flash_agnos_update(manifest_path, target_slot_number, cloudlog)
  set_offroad_alert("Offroad_NeosUpdate", False)


def handle_neos_update() -> None:


  cur_neos = HARDWARE.get_os_version()
  updated_neos = run(["bash", "-c", r"unset REQUIRED_NEOS_VERSION && source launch_env.sh && \
                       echo -n $REQUIRED_NEOS_VERSION"], OVERLAY_MERGED).strip()

  cloudlog.info(f"NEOS version check: {cur_neos} vs {updated_neos}")
  if cur_neos == updated_neos:
    return

  cloudlog.info(f"Beginning background download for NEOS {updated_neos}")
  set_offroad_alert("Offroad_NeosUpdate", True)

  manifest_path = os.path.join(OVERLAY_MERGED, "system/hardware/eon/neos.json")
  download_neos_update(manifest_path, cloudlog)
  set_offroad_alert("Offroad_NeosUpdate", False)

class Updater:
  def __init__(self):
    self.params = Params()
    self.branches = defaultdict(str)
    self._has_internet: bool = False

  @property
  def has_internet(self) -> bool:
    return self._has_internet

  @property
  def target_branch(self) -> str:
    b: Optional[str] = self.params.get("UpdaterTargetBranch", encoding='utf-8')
    if b is None:
      b = self.get_branch(BASEDIR)
    return b

  @property
  def update_ready(self) -> bool:
    consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
    if consistent_file.is_file():
      hash_mismatch = self.get_commit_hash(BASEDIR) != self.branches[self.target_branch]
      branch_mismatch = self.get_branch(BASEDIR) != self.target_branch
      on_target_branch = self.get_branch(FINALIZED) == self.target_branch
      return ((hash_mismatch or branch_mismatch) and on_target_branch)
    return False

  @property
  def update_available(self) -> bool:
    if os.path.isdir(OVERLAY_MERGED) and len(self.branches) > 0:
      hash_mismatch = self.get_commit_hash(OVERLAY_MERGED) != self.branches[self.target_branch]
      branch_mismatch = self.get_branch(OVERLAY_MERGED) != self.target_branch
      return hash_mismatch or branch_mismatch
    return False

  def get_branch(self, path: str) -> str:
    return run(["git", "rev-parse", "--abbrev-ref", "HEAD"], path).rstrip()

  def get_commit_hash(self, path: str = OVERLAY_MERGED) -> str:
    return run(["git", "rev-parse", "HEAD"], path).rstrip()

  def set_params(self, update_success: bool, failed_count: int, exception: Optional[str]) -> None:
    self.params.put("UpdateFailedCount", str(failed_count))
    self.params.put("UpdaterTargetBranch", self.target_branch)

    self.params.put_bool("UpdaterFetchAvailable", self.update_available)
    if len(self.branches):
      self.params.put("UpdaterAvailableBranches", ','.join(self.branches.keys()))

    last_update = datetime.datetime.utcnow()
    if update_success:
      write_time_to_param(self.params, "LastUpdateTime")
    else:
      t = read_time_from_param(self.params, "LastUpdateTime")
      if t is not None:
        last_update = t

    if exception is None:
      self.params.remove("LastUpdateException")
    else:
      self.params.put("LastUpdateException", exception)

    # Write out current and new version info
    def get_description(basedir: str) -> str:
      if not os.path.exists(basedir):
        return ""

      version = ""
      branch = ""
      commit = ""
      commit_date = ""
      try:
        branch = self.get_branch(basedir)
        commit = self.get_commit_hash(basedir)[:7]
        with open(os.path.join(basedir, "common", "version.h")) as f:
          version = f.read().split('"')[1]

        commit_unix_ts = run(["git", "show", "-s", "--format=%ct", "HEAD"], basedir).rstrip()
        dt = datetime.datetime.fromtimestamp(int(commit_unix_ts))
        commit_date = dt.strftime("%b %d")
      except Exception:
        cloudlog.exception("updater.get_description")
      return f"{version} / {branch} / {commit} / {commit_date}"
    self.params.put("UpdaterCurrentDescription", get_description(BASEDIR))
    self.params.put("UpdaterCurrentReleaseNotes", parse_release_notes(BASEDIR))
    self.params.put("UpdaterNewDescription", get_description(FINALIZED))
    self.params.put("UpdaterNewReleaseNotes", parse_release_notes(FINALIZED))
    self.params.put_bool("UpdateAvailable", self.update_ready)

    # Handle user prompt
    for alert in ("Offroad_UpdateFailed", "Offroad_ConnectivityNeeded", "Offroad_ConnectivityNeededPrompt"):
      set_offroad_alert(alert, False)

    now = datetime.datetime.utcnow()
    dt = now - last_update
    if failed_count > 15 and exception is not None and self.has_internet:
      if is_tested_branch():
        extra_text = "Ensure the software is correctly installed. Uninstall and re-install if this error persists."
      else:
        extra_text = exception
      set_offroad_alert("Offroad_UpdateFailed", True, extra_text=extra_text)
    # elif failed_count > 0:
    #   if dt.days > DAYS_NO_CONNECTIVITY_MAX:
    #     set_offroad_alert("Offroad_ConnectivityNeeded", True)
    #   elif dt.days > DAYS_NO_CONNECTIVITY_PROMPT:
    #     remaining = max(DAYS_NO_CONNECTIVITY_MAX - dt.days, 1)
    #     set_offroad_alert("Offroad_ConnectivityNeededPrompt", True, extra_text=f"{remaining} day{'' if remaining == 1 else 's'}.")

  def check_for_update(self) -> None:
    cloudlog.info("checking for updates")

    excluded_branches = ('beta3', 'r3', 'd3')

    try:
      run(["git", "ls-remote", "origin", "HEAD"], OVERLAY_MERGED)
      self._has_internet = True
    except subprocess.CalledProcessError:
      self._has_internet = False

    setup_git_options(OVERLAY_MERGED)
    output = run(["git", "ls-remote", "--heads"], OVERLAY_MERGED)

    self.branches = defaultdict(lambda: None)
    for line in output.split('\n'):
      ls_remotes_re = r'(?P<commit_sha>\b[0-9a-f]{5,40}\b)(\s+)(refs\/heads\/)(?P<branch_name>.*$)'
      x = re.fullmatch(ls_remotes_re, line.strip())
      if x is not None and x.group('branch_name') not in excluded_branches:
        self.branches[x.group('branch_name')] = x.group('commit_sha')

    cur_branch = self.get_branch(OVERLAY_MERGED)
    cur_commit = self.get_commit_hash(OVERLAY_MERGED)
    new_branch = self.target_branch
    new_commit = self.branches[new_branch]
    if (cur_branch, cur_commit) != (new_branch, new_commit):
      cloudlog.info(f"update available, {cur_branch} ({str(cur_commit)[:7]}) -> {new_branch} ({str(new_commit)[:7]})")
    else:
      cloudlog.info(f"up to date on {cur_branch} ({str(cur_commit)[:7]})")

  def fetch_update(self) -> None:
    cloudlog.info("attempting git fetch inside staging overlay")
    self.params.put("UpdaterState", "preparing...")

    # 设置 Git 配置
    setup_git_options(OVERLAY_MERGED)
    
    # 准备更新
    set_consistent_flag(False)
    self.params.put_bool("UpdateAvailable", False)
    branch = self.target_branch

    # 使用 Popen 来获取实时进度
    fetch_cmd = ["git", "fetch", "--progress", "origin", branch]
    process = subprocess.Popen(
        fetch_cmd,
        cwd=OVERLAY_MERGED,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True
    )

    # 读取进度信息
    while True:
        stderr = process.stderr.readline() if process.stderr else ""
        if stderr == "" and process.poll() is not None:
            break
        if stderr:
            # 解析进度信息
            if "Receiving objects:" in stderr:
                try:
                    progress = stderr.split()[2].rstrip("%")
                    self.params.put("UpdaterState", f"downloading {progress}%")
                except:
                    cloudlog.exception("failed to parse git progress")
            cloudlog.info(f"git fetch progress: {stderr.strip()}")

    if process.returncode != 0:
        raise subprocess.CalledProcessError(process.returncode, fetch_cmd)

    # 继续其他更新步骤
    self.params.put("UpdaterState", "finalizing...")
    cloudlog.info("git fetch completed, processing update...")

    # 执行其他 git 命令
    cmds = [
        ["git", "checkout", "--force", "--no-recurse-submodules", "-B", branch, "FETCH_HEAD"],
        ["git", "reset", "--hard"],
        ["git", "clean", "-xdff"],
        ["git", "submodule", "sync"],
        ["git", "submodule", "update", "--init", "--recursive"],
        ["git", "submodule", "foreach", "--recursive", "git", "reset", "--hard"],
    ]
    
    for cmd in cmds:
        cloudlog.info(f"Running: {' '.join(cmd)}")
        run(cmd, OVERLAY_MERGED)

    # 处理系统更新
    if EON and not os.path.isfile("/ONEPLUS"):
        handle_neos_update()
    elif AGNOS:
        handle_agnos_update()

    # 完成更新
    self.params.put("UpdaterState", "finalizing update...")
    finalize_update()
    cloudlog.info("finalize success!")


def main() -> None:
  params = Params()

  if params.get_bool("DisableUpdates"):
    cloudlog.info("updates are disabled by the DisableUpdates param")
    exit(0)

  with open(LOCK_FILE, 'w') as ov_lock_fd:
    try:
      fcntl.flock(ov_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
      raise RuntimeError("couldn't get overlay lock; is another instance running?") from e

    # Set low io priority
    proc = psutil.Process()
    if psutil.LINUX:
      proc.ionice(psutil.IOPRIO_CLASS_BE, value=7)

    # Check if we just performed an update
    if Path(os.path.join(STAGING_ROOT, "old_openpilot")).is_dir():
      cloudlog.event("update installed")

    if not params.get("InstallDate"):
      t = datetime.datetime.utcnow().isoformat()
      params.put("InstallDate", t.encode('utf8'))

    updater = Updater()
    update_failed_count = 0 # TODO: Load from param?
    wait_helper = WaitTimeHelper()

    # invalidate old finalized update
    set_consistent_flag(False)

    # set initial state
    params.put("UpdaterState", "idle")

    # Run the update loop
    first_run = True
    while True:
      wait_helper.ready_event.clear()

      # Attempt an update
      exception = None
      try:
        # TODO: reuse overlay from previous updated instance if it looks clean
        init_overlay()

        # ensure we have some params written soon after startup
        updater.set_params(False, update_failed_count, exception)

        if not system_time_valid() or first_run:
          first_run = False
          wait_helper.sleep(60)
          continue

        update_failed_count += 1

        # check for update
        params.put("UpdaterState", "checking...")
        updater.check_for_update()

        # download update
        last_fetch = read_time_from_param(params, "UpdaterLastFetchTime")
        timed_out = last_fetch is None or (datetime.datetime.utcnow() - last_fetch > datetime.timedelta(days=3))
        user_requested_fetch = wait_helper.user_request == UserRequest.FETCH
        if params.get_bool("NetworkMetered") and not timed_out and not user_requested_fetch:
          cloudlog.info("skipping fetch, connection metered")
        elif wait_helper.user_request == UserRequest.CHECK:
          cloudlog.info("skipping fetch, only checking")
        else:
          updater.fetch_update()
          write_time_to_param(params, "UpdaterLastFetchTime")
        update_failed_count = 0
      except subprocess.CalledProcessError as e:
        cloudlog.event(
          "update process failed",
          cmd=e.cmd,
          output=e.output,
          returncode=e.returncode
        )
        exception = f"command failed: {e.cmd}\n{e.output}"
        OVERLAY_INIT.unlink(missing_ok=True)
      except Exception as e:
        cloudlog.exception("uncaught updated exception, shouldn't happen")
        exception = str(e)
        OVERLAY_INIT.unlink(missing_ok=True)

      try:
        params.put("UpdaterState", "idle")
        update_successful = (update_failed_count == 0)
        updater.set_params(update_successful, update_failed_count, exception)
      except Exception:
        cloudlog.exception("uncaught updated exception while setting params, shouldn't happen")

      # infrequent attempts if we successfully updated recently
      wait_helper.user_request = UserRequest.NONE
      wait_helper.sleep(5*60 if update_failed_count > 0 else 1.5*60*60)


if __name__ == "__main__":
  main()
