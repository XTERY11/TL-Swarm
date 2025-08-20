#!/usr/bin/env python3
import os, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bag_dir = '/home/ctx/hifisim_ws/cameras_bag2'          # bag 目录
topics  = {                                            # 需要导出的多路
    '/agent002/camera01_left' : 'imgs_agent002_left',
    '/agent002/camera01_right': 'imgs_agent002_right',
}

for folder in topics.values():
    os.makedirs(folder, exist_ok=True)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', '')
)
的
bridge = CvBridge()
counters = {t:0 for t in topics}                       # 每路计数
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic in topics:
        img_msg = deserialize_message(data, Image)
        cv_img  = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        idx     = counters[topic]
        cv2.imwrite(f"{topics[topic]}/{idx:06d}.png", cv_img)
        counters[topic] += 1

print('保存完毕：')
for t, n in counters.items():
    print(f'  {t}  →  {n} 张')