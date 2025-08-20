#!/usr/bin/env bash
set -euo pipefail

# Config
UNITY_BIN="/home/ctx/Unity/build/hifi_simulator_unity.x86_64"
ENDPOINT_CMD=(ros2 run tcp_endpoint_ros2 default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000)
UNITY_LOG="/tmp/unity_detailed.log"
ENDPOINT_LOG="/tmp/ros_tcp_endpoint_detailed.log"
DISPLAY_VAL="${DISPLAY:-:0}"
# Allow override via env or CLI
HEADLESS=${HEADLESS:-0}

# Parse CLI flags
for arg in "$@"; do
	case "$arg" in
		--headless)
			HEADLESS=1
			shift || true
			;;
	esac
done

banner() { echo -e "\n==== $* ====\n"; }

kill_all() {
	pkill -9 -f "hifi_simulator_unity" || true
	pkill -9 -f "default_server_endpoint" || true
	pkill -9 -f "sim_gcs" || true
}

clean_ros() {
	ros2 daemon stop || true
	sleep 1
	ros2 daemon start || true
	find /dev/shm -maxdepth 1 -type s -name 'fastrtps_*' -delete 2>/dev/null || true
	rm -rf ~/.ros/log/* 2>/dev/null || true
}

start_xvfb() {
	banner "No usable DISPLAY found. Starting Xvfb :99"
	if ! command -v Xvfb >/dev/null 2>&1; then
		echo "Xvfb not installed. Please install 'xvfb'." >&2
		return 1
	fi
	# Ensure runtime dir for X
	export XDG_RUNTIME_DIR="/tmp/runtime-${USER}"
	mkdir -p "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" || true
	# Start headless X server
	Xvfb :99 -screen 0 1920x1080x24 -ac -nolisten tcp >/tmp/xvfb.log 2>&1 &
	XVFB_PID=$!
	sleep 1
	if ! ps -p ${XVFB_PID} >/dev/null 2>&1; then
		echo "Failed to start Xvfb. See /tmp/xvfb.log"
		return 1
	fi
	export DISPLAY=":99"
	DISPLAY_VAL=":99"
	HEADLESS=1
	return 0
}

ensure_display() {
	# If DISPLAY unset or unreachable, fall back to Xvfb
	if ! timeout 1 bash -lc "xdpyinfo -display ${DISPLAY_VAL} >/dev/null 2>&1"; then
		start_xvfb || return 1
	else
		# Allow local clients to connect (best-effort)
		if command -v xhost >/dev/null 2>&1; then xhost +local: >/dev/null 2>&1 || true; fi
	fi
	return 0
}

start_unity() {
	banner "Starting Unity (DISPLAY=${DISPLAY_VAL}, HEADLESS=${HEADLESS})"
	: >"${UNITY_LOG}"
	# Safer defaults; if headless, force software GL and no graphics
	export XDG_RUNTIME_DIR="/tmp/runtime-${USER}"
	mkdir -p "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" || true
	# Prefer X11 over Wayland; reduce driver variance
	export XDG_SESSION_TYPE=x11
	export SDL_VIDEODRIVER=x11
	export QT_QPA_PLATFORM=xcb
	if [[ ${HEADLESS} -eq 1 ]]; then
		export LIBGL_ALWAYS_SOFTWARE=1
		UNITY_ARGS=("-batchmode" "-nographics" "-screen-width" "1280" "-screen-height" "720" "-logFile" "${UNITY_LOG}")
	else
		# Force OpenGL core to avoid Vulkan swapchain crashes
		UNITY_ARGS=("-force-glcore" "-logFile" "${UNITY_LOG}")
	fi
	DISPLAY="${DISPLAY_VAL}" nohup "${UNITY_BIN}" "${UNITY_ARGS[@]}" >>"${UNITY_LOG}" 2>&1 &
	UNITY_PID=$!
	echo "UNITY_PID=${UNITY_PID}"
	sleep 5
	if ! ps -p ${UNITY_PID} >/dev/null 2>&1; then
		echo "Unity exited quickly. Retrying once with headless mode..."
		export LIBGL_ALWAYS_SOFTWARE=1
		DISPLAY_VAL=":99"; start_xvfb || true
		UNITY_ARGS=("-batchmode" "-nographics" "-screen-width" "1280" "-screen-height" "720" "-logFile" "${UNITY_LOG}")
		nohup "${UNITY_BIN}" "${UNITY_ARGS[@]}" >>"${UNITY_LOG}" 2>&1 &
		UNITY_PID=$!
		echo "UNITY_PID=${UNITY_PID} (retry)"
		sleep 5
		if ! ps -p ${UNITY_PID} >/dev/null 2>&1; then
			echo "Unity failed to stay up. Check ${UNITY_LOG}"
			tail -n 80 "${UNITY_LOG}" || true
			return 1
		fi
	fi
	return 0
}

start_endpoint() {
	banner "Starting TCP Endpoint"
	: >"${ENDPOINT_LOG}"
	nohup "${ENDPOINT_CMD[@]}" >"${ENDPOINT_LOG}" 2>&1 &
	ENDPOINT_PID=$!
	echo "ENDPOINT_PID=${ENDPOINT_PID}"
	sleep 2
	if ! ps -p ${ENDPOINT_PID} >/dev/null 2>&1; then
		echo "Endpoint died. Check ${ENDPOINT_LOG}"
		return 1
	fi
	return 0
}

wait_services() {
	banner "Waiting for Unity services (/init_testcase)"
	if timeout 20 bash -c 'until ros2 service list | grep -q "^/init_testcase$"; do sleep 0.5; done'; then
		echo "Services ready"
		return 0
	else
		echo "Services NOT ready. Logs:"
		tail -n 40 "${UNITY_LOG}" || true
		tail -n 40 "${ENDPOINT_LOG}" || true
		return 1
	fi
}

run_gcs() {
	banner "Starting sim_gcs (follow prompts)"
	ros2 run hifisim_task_ros2 sim_gcs
}

main() {
	banner "Clean previous processes"
	kill_all
	clean_ros

	ensure_display || exit 1
	start_unity || exit 1
	start_endpoint || exit 1
	wait_services || exit 1
	run_gcs
}

main "$@" 