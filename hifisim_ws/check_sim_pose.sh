#!/bin/bash
set -euo pipefail

# Usage: ./check_sim_pose.sh N
# Check /agentXXX/global/sim_nwu_pose for agent001..agent00N (timeout 2s each)
if [ $# -lt 1 ]; then
	echo "Usage: $0 NUM_AGENTS" >&2
	exit 1
fi

NUM=$1
OK=0
for ((i=1;i<=NUM;i++)); do
	A=$(printf "%03d" "$i")
	TOPIC="/agent${A}/global/sim_nwu_pose"
	echo "Check ${TOPIC} ..."
	if timeout 2 ros2 topic echo ${TOPIC} --once >/dev/null 2>&1; then
		echo "  ${TOPIC}: OK"
		OK=$((OK+1))
	else
		echo "  ${TOPIC}: NO MSG"
	fi
done

echo "Summary: ${OK}/${NUM} agents have sim_nwu_pose"
exit 0 