# scripts/multi_upload.py
# Upload over USB or OTA to multiple or individual targets.
# Also adds per-port Serial Monitor targets.
Import("env")
import subprocess

# ---------- USB COM ports ----------
USB_PORTS = ["COM7", "COM5", "COM4", "COM6"]

# All USB uploads use the same build env:
USB_BUILD_ENV = "esp32s3"

# ---------- OTA targets (env_name, label) ----------
OTA_ENVS = [
    ("robot01_ota", "robot-01 (192.168.86.101)"),
    ("robot02_ota", "robot-02 (192.168.86.102)"),
    ("robot03_ota", "robot-03 (192.168.86.103)"),
    ("robot04_ota", "robot-04 (192.168.86.104)"),
]

# Serial monitor speed (match platformio.ini monitor_speed)
MONITOR_BAUD = "115200"

def _run(cmd):
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        print("*** Failed to execute:", " ".join(cmd))
        return 1

# ---- helpers ----
def _pio_build_usb_firmware(env_for_build, env_handle):
    """Always ensure the USB firmware is built once before uploads,
    regardless of which env invoked the custom target."""
    py = env_handle.subst("$PYTHONEXE")
    print(f"=== [USB] Building firmware for env {env_for_build} ===")
    return _run([py, "-m", "platformio", "run", "-e", env_for_build])

# ---- Actions (must accept target, source, env for SCons) ----
def pio_usb_upload_to(port):
    def _action(target, source, env):
        # Build once (idempotent / cached)
        rc = _pio_build_usb_firmware(USB_BUILD_ENV, env)
        if rc != 0:
            print("*** USB build failed; aborting uploads")
            return rc
        py = env.subst("$PYTHONEXE")
        print(f"=== [USB] Uploading {USB_BUILD_ENV} -> {port} ===")
        cmd = [py, "-m", "platformio", "run",
               "-e", USB_BUILD_ENV,
               "-t", "upload",
               "-t", "nobuild",
               "--upload-port", port]
        return _run(cmd)
    return _action

def multi_upload_usb(target, source, env):
    # Build once up-front, then fast uploads with -t nobuild
    rc = _pio_build_usb_firmware(USB_BUILD_ENV, env)
    if rc != 0:
        print("*** USB build failed; aborting uploads")
        return rc
    for p in USB_PORTS:
        rc |= pio_usb_upload_to(p)(target, source, env)
    return 1 if rc != 0 else 0

def pio_ota_upload_env(env_name):
    def _action(target, source, env):
        py = env.subst("$PYTHONEXE")
        print(f"=== [OTA] Uploading {env_name} ===")
        # Build per env (ROBOT_NAME/IP differ)
        cmd = [py, "-m", "platformio", "run",
               "-e", env_name,
               "-t", "upload"]
        return _run(cmd)
    return _action

def multi_upload_ota(target, source, env):
    rc = 0
    for env_name, _label in OTA_ENVS:
        rc |= pio_ota_upload_env(env_name)(target, source, env)
    return 1 if rc != 0 else 0

# ---- Serial monitor per port ----
def pio_monitor_port(port):
    def _action(target, source, env):
        py = env.subst("$PYTHONEXE")
        print(f"=== [MON] Opening serial monitor on {port} @ {MONITOR_BAUD} ===")
        cmd = [py, "-m", "platformio", "device", "monitor",
               "--port", port, "--baud", MONITOR_BAUD]
        return _run(cmd)   # blocking until closed
    return _action

# ---- Register targets ----
# USB: Upload to all four
env.AddCustomTarget(
    name="usb_all",
    dependencies=[],                   # build is handled inside the action
    actions=[multi_upload_usb],
    title="USB: Upload to ALL boards",
    description=f"Build once, upload over USB to: {', '.join(USB_PORTS)}"
)

# USB: per-port
for p in USB_PORTS:
    env.AddCustomTarget(
        name=f"usb_{p}",
        dependencies=[],
        actions=[pio_usb_upload_to(p)],
        title=f"USB: Upload {p} only",
        description=f"Build once (if needed), upload over USB to {p}"
    )

# OTA: all four
env.AddCustomTarget(
    name="ota_all",
    dependencies=[],
    actions=[multi_upload_ota],
    title="OTA: Upload to ALL boards",
    description="Compile/upload per-robot over Wi-Fi"
)

# OTA: per-robot
for env_name, label in OTA_ENVS:
    env.AddCustomTarget(
        name=f"ota_{env_name}",
        dependencies=[],
        actions=[pio_ota_upload_env(env_name)],
        title=f"OTA: Upload {label}",
        description=f"Compile/upload OTA to {label}"
    )

# Monitors
for p in USB_PORTS:
    env.AddCustomTarget(
        name=f"mon_{p}",
        dependencies=[],
        actions=[pio_monitor_port(p)],
        title=f"MON: Open Serial {p}",
        description=f"Serial monitor on {p} at {MONITOR_BAUD} baud"
    )

def show_monitor_hint(target, source, env):
    print("Note: Only one monitor can own a COM port at a time.")
    print("Close the monitor (Ctrl+C) before uploading to that board.")
    return 0

env.AddCustomTarget(
    name="mon_CLOSE_HINT",
    dependencies=[],
    actions=[show_monitor_hint],
    title="MON: Close monitors before upload",
    description="Reminder: monitors are exclusive; close before uploading."
)
