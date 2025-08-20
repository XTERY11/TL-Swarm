set -e

REQ_NODES=(/UnityEndpoint /performance_node /radio_comm_simulator)

echo "check核心节点上线状态..."
for node in "${REQ_NODES[@]}"; do
  until ros2 node list | grep -q "$node"; do
    echo "  $node 未就绪，继续等待..."
    sleep 1
  done
  echo "  $node 已上线"
done
echo ">>> 所有关键节点已全部 Online"