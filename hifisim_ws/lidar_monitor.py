# 


from hifisim_task_ros2.read_testcase import get_signposts_endpoint_nwu_pos_rot_env, get_signposts_nwu_pos
pts_rot = get_signposts_endpoint_nwu_pos_rot_env("Env1", "1")
signposts = get_signposts_nwu_pos(pts_rot)  # 长度=9（不含最后的 endpoint）
for i, p in enumerate(signposts, 1):
    print(f"Signpost_{i}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")