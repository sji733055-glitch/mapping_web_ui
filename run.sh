#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_WORKSPACE="${MAS_NAV_WORKSPACE:-/home/mas/mas_nav_2027_native}"
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
set -u

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
        if ! open_console_url "${CONSOLE_URL}"; then
          echo "未检测到可用的浏览器打开方式，请手动访问 ${CONSOLE_URL}" >&2
        fi
        exit 0
      fi
      sleep 0.25
    done
    echo "后端启动后未能自动打开网页，请手动访问 ${CONSOLE_URL}" >&2
  ) &
else
  echo "自动打开浏览器已关闭；服务启动后请访问 ${CONSOLE_URL}"
fi

exec python3 "${PROJECT_ROOT}/backend/mapping_server.py" "${BACKEND_ARGS[@]}"
