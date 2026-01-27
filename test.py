import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()
profile = pipeline.start(config)
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
# 获取深度传感器
depth_sensor = profile.get_device().first_depth_sensor()

# 获取缩放因子
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale (m per unit):", depth_scale)
print(intrinsics)
pipeline.stop()