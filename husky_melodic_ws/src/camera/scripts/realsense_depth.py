import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        # self.pipeline.start(config)
        self.profile = self.pipeline.start(config)

        # Get depth intrinsics
        depth_profile = pipeline_profile.get_stream(rs.stream.depth)
        self.depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def pixel_to_point(self, depth_frame, pixel):
        profile = self.profile
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        depth_value = depth_frame[pixel[1], pixel[0]]
        # point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [pixel[0], pixel[1]], depth_value)
        ppx = intr.ppx
        ppy = intr.ppy
        fx = intr.fx
        fy = intr.fy
        print("ppx",ppx)
        print("ppy",ppy)
        print("fx",fx)
        print("fy",fy)
        print("=======================")
        
        point = [0,0,0]
        point[0] = depth_value*(pixel[0]+4 - ppx)/fx
        point[1] = depth_value*(pixel[1]+8 - ppy)/fy
        point[2] = depth_value

        return point

    def release(self):
        self.pipeline.stop()
