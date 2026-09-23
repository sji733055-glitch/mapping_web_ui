#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_WORKSPACE="${MAS_NAV_WORKSPACE:-/home/mas/mas_nav_2027_native}"
LOCAL_ROS_WORKSPACE="${PROJECT_ROOT}/ros2_ws"
OPEN_BROWSER=true
WEB_HOST="0.0.0.0"
WEB_PORT="8765"
BACKEND_ARGS=()

while (($#)); do
  case "$1" in
    --no-browser)
      OPEN_BROWSER=false
      shift
      ;;
    --host)
      if (($# < 2)); then
        echo "--host 需要一个地址参数。" >&2
        exit 2
      fi
      WEB_HOST="$2"
      BACKEND_ARGS+=("$1" "$2")
      shift 2
      ;;
    --host=*)
      WEB_HOST="${1#--host=}"
      BACKEND_ARGS+=("$1")
      shift
      ;;
    --port)
      if (($# < 2)); then
        echo "--port 需要一个端口参数。" >&2
        exit 2
      fi
      WEB_PORT="$2"
      BACKEND_ARGS+=("$1" "$2")
      shift 2
      ;;
    --port=*)
      WEB_PORT="${1#--port=}"
      BACKEND_ARGS+=("$1")
      shift
      ;;
    *)
      BACKEND_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "找不到 /opt/ros/jazzy/setup.bash，请先安装 ROS 2 Jazzy。" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "${NAV_WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${NAV_WORKSPACE}/install/setup.bash"
fi
if [[ -f "${LOCAL_ROS_WORKSPACE}/install/setup.bash" ]]; then
  # Source last so a locally built small_point_lio overrides the navigation
  # workspace package without modifying that workspace.
  # shellcheck disable=SC1090
  source "${LOCAL_ROS_WORKSPACE}/install/setup.bash"
fi
set -u

# 海康 MVS 自带一份较旧的 libusb-1.0.so.0（不含 libusb_set_option），而
# /etc/profile 与 ~/.bashrc 会把 /opt/MVS/lib/64 前置进 LD_LIBRARY_PATH。任何加载
# 系统 libpcl_io 的进程（受管建图链、隔离轨迹实验室的 nav_executor）都会以
# `undefined symbol: libusb_set_option` 退出。LD_LIBRARY_PATH 是整体先于 ld.so
# 缓存搜索的，所以把 MVS 改成追加也没用，必须把系统目录显式排在它前面。
# 只影响本脚本启动的后端及其受管子进程；单独启动的 MVS 客户端不受影响。
# 系统 libusb 是 MVS 那份的严格超集（多出 libusb_set_option / libusb_init_context
# 等标准 API），因此 MVS 的 USB3 传输层改用它仍可正常加载与枚举设备。
SYSTEM_LIB_DIR="/usr/lib/x86_64-linux-gnu"
MVS_LIBUSB="/opt/MVS/lib/64/libusb-1.0.so.0"
if [[ -d "${SYSTEM_LIB_DIR}" && -e "${MVS_LIBUSB}" ]]; then
  if [[ "${LD_LIBRARY_PATH:-}" != "${SYSTEM_LIB_DIR}" && "${LD_LIBRARY_PATH:-}" != "${SYSTEM_LIB_DIR}:"* ]]; then
    export LD_LIBRARY_PATH="${SYSTEM_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    echo "已把 ${SYSTEM_LIB_DIR} 置于 LD_LIBRARY_PATH 首位，避免海康 MVS 的旧 libusb 遮蔽系统 libusb。" >&2
  fi
fi

mkdir -p "${PROJECT_ROOT}/.ros/log" "${PROJECT_ROOT}/data/pcd" "${PROJECT_ROOT}/data/map"
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros/log"
export PYTHONUNBUFFERED=1

URL_HOST="${WEB_HOST}"
if [[ "${URL_HOST}" == "0.0.0.0" || "${URL_HOST}" == "::" || -z "${URL_HOST}" ]]; then
  URL_HOST="127.0.0.1"
elif [[ "${URL_HOST}" == *:* && "${URL_HOST}" != \[*\] ]]; then
  URL_HOST="[${URL_HOST}]"
fi
CONSOLE_URL="http://${URL_HOST}:${WEB_PORT}"

# 浏览器通常不在本机运行（VS Code Remote SSH 转发或局域网直连）。复制 127.0.0.1
# 到别的电脑上只会指向那台电脑自己，所以这里额外给出本机在局域网中的地址。
LAN_URL=""
if [[ -z "${WEB_HOST}" || "${WEB_HOST}" == "0.0.0.0" || "${WEB_HOST}" == "::" ]]; then
  SSH_PEER="${SSH_CONNECTION:-}"
  SSH_PEER="${SSH_PEER%% *}"
  if [[ -n "${SSH_PEER}" ]] && command -v ip >/dev/null 2>&1; then
    LAN_IP="$(ip route get "${SSH_PEER}" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") { print $(i + 1); exit }}')"
    if [[ -n "${LAN_IP}" ]]; then
      LAN_URL="http://${LAN_IP}:${WEB_PORT}"
    fi
  fi
fi

print_access_hint() {
  if [[ -n "${LAN_URL}" ]]; then
    echo "同一网络的其他电脑请访问：${LAN_URL}（复制 ${CONSOLE_URL} 只有这台机器自己能打开）"
  fi
}

open_console_url() {
  local url="$1"
  if [[ -n "${VSCODE_IPC_HOOK_CLI:-}" ]] && command -v code >/dev/null 2>&1; then
    if code --openExternal "${url}" >/dev/null 2>&1; then
      return 0
    fi
  fi
  if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "${url}" >/dev/null 2>&1 && return 0
    fi
    if command -v gio >/dev/null 2>&1; then
      gio open "${url}" >/dev/null 2>&1 && return 0
    fi
    if command -v sensible-browser >/dev/null 2>&1; then
      sensible-browser "${url}" >/dev/null 2>&1 && return 0
    fi
  fi
  return 1
}

if [[ "${OPEN_BROWSER}" == true ]]; then
  (
    STATUS_URL="${CONSOLE_URL}/api/status"
    for _attempt in {1..60}; do
      if python3 -c 'import sys, urllib.request; urllib.request.urlopen(sys.argv[1], timeout=0.4).read(1)' "${STATUS_URL}" >/dev/null 2>&1; then
        echo "网页已就绪：${CONSOLE_URL}"
        print_access_hint
        if ! open_console_url "${CONSOLE_URL}"; then
          echo "未检测到可用的浏览器打开方式，请手动访问 ${CONSOLE_URL}" >&2
          print_access_hint
        fi
        exit 0
      fi
      sleep 0.25
    done
    echo "后端启动后未能自动打开网页，请手动访问 ${CONSOLE_URL}" >&2
    print_access_hint
  ) &
else
  echo "自动打开浏览器已关闭；服务启动后请访问 ${CONSOLE_URL}"
  print_access_hint
fi

exec python3 "${PROJECT_ROOT}/backend/mapping_server.py" "${BACKEND_ARGS[@]}"
