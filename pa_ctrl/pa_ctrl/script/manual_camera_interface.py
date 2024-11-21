#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import message_filters
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import tf2_ros
import tf2_geometry_msgs
import ros_numpy
from opencv_apps.msg import RotatedRect


class ImageInterface:

    def __init__(self) -> None:
        
        ### IMAGE PIPELINE ----------------------
        #self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callbackRGB, queue_size=10)

        #if rospy.get_param("/sensor_used/offline"):
        self.img_dim_x = 1280
        self.img_dim_y = 720
        #else:
        #    self.img_dim_x = rospy.get_param("/camera/realsense2_camera/color_width")
        #    self.img_dim_y = rospy.get_param("/camera/realsense2_camera/color_height")
        ### --------------------------------------

        ### Camera calibration -------------------
        # self.camera_model = PinholeCameraModel()
        # self.camera_resolutionX = 1.0  # mm/pixel
        # self.camera_resolutionY = 1.0  # mm/pixel
        # self.info_image_3d = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=1)
        # self.info_image_3d.registerCallback(self.callbackInfoCam)

        # self.last_cv_image_rectified = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint8)
        ### --------------------------------------

        ### POINTCLOUD PIPELINE ------------------
        # self.pc_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callbackPC, queue_size=10) ###oganised point cloud avec rgbd_launch.launch.xml
        # self.pc_raw = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)
        ### --------------------------------------

        ### Init de claude 3 ---------------------
        rospy.init_node('realsense_camera')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.CallbackImage)
        #self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.CallbackPointCloud)
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.click_point = None
        self.drag_point = None
        #self.pc_raw = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)

        ### --------------------------------------
        ### --------------------------------------
        self.box_pub = rospy.Publisher('/bounding_box',RotatedRect,queue_size=10)
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0
        ### --------------------------------------


    def CallbackImage(self, data):
        ### from claude 3
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.click_point is not None and self.drag_point is not None:
                cv2.line(cv_image, self.click_point, self.drag_point, (0, 255, 0), 2)
            cv2.imshow("RealSense Camera", cv_image)
            cv2.setMouseCallback("RealSense Camera", self.mouse_callback)
            cv2.waitKey(1)
        except Exception as e:
            print("Callback Image exception")
            print(e)

    # def CallbackPointCloud(self,data):
    #     try:
    #         if self.click_point is not None and self.drag_point is not None:
    #             #point_cloud = self.bridge.imgmsg_to_cv2(data, "passthrough")
    #             point_cloud = data
    #             point_cloud_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    #             click_point_3d = self.get_3d_point(self.click_point[0], self.click_point[1], point_cloud)
    #             drag_point_3d = self.get_3d_point(self.drag_point[0], self.drag_point[1], point_cloud)
    #             print(f"Click point: {click_point_3d}")
    #             print(f"Drag point: {drag_point_3d}")
    #     except Exception as e:
    #         print("Callback Image Point Cloud")
    #         print(e)

    # def callbackPC(self,data):
    #     self.pc_raw, self.pc_dtype, self.pc_header_raw = self.rospc_to_nppc(data)


    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_LBUTTON:
            self.drag_point = (x, y)
            self.publish_box()
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_point = (x, y)
            self.publish_box()
    
    def publish_box(self):
        box_msg = RotatedRect()
        box_msg.center.x = (self.click_point[0] + self.drag_point[0])/2
        box_msg.center.y = (self.click_point[1] + self.drag_point[1])/2
        box_msg.size.width = abs(self.drag_point[0] - self.click_point[0])
        box_msg.size.height= abs(self.drag_point[1] - self.click_point[1])
        box_msg.angle = 0
        print(box_msg)
        self.box_pub.publish(box_msg)
        
        

    # def get_3d_pointARCHIVE(self, x, y, point_cloud):
    #     try:
    #         #transform = self.tf_buffer.lookup_transform('camera_link', 'camera_color_optical_frame', rospy.Time(0))
    #         point_cloud_fields = point_cloud.fields
    #         point_cloud_data   = point_cloud.data
    #         point_cloud_step   = point_cloud.point_step
    #         point_cloud_row_step = point_cloud.width

    #         # Find the indices of the x, y, and z fields
    #         x_idx = next((i for i, f in enumerate(point_cloud_fields) if f.name == 'x'), None)
    #         y_idx = next((i for i, f in enumerate(point_cloud_fields) if f.name == 'y'), None)
    #         z_idx = next((i for i, f in enumerate(point_cloud_fields) if f.name == 'z'), None)

    #         if x_idx is None or y_idx is None or z_idx is None:
    #             return (0, 0, 0)

    #         # Calculate the index of the point at (x, y)
    #         point_idx = int(y * point_cloud_row_step + x * point_cloud_step)

    #         # Extract the x, y, and z values of the point
    #         x_value = np.frombuffer(point_cloud_data[point_idx + x_idx:point_idx + x_idx + 4], dtype=np.float32)
    #         y_value = np.frombuffer(point_cloud_data[point_idx + y_idx:point_idx + y_idx + 4], dtype=np.float32)[0]
    #         z_value = np.frombuffer(point_cloud_data[point_idx + z_idx:point_idx + z_idx + 4], dtype=np.float32)[0]

    #         # Transform the point to the 'camera_link' frame
    #         point = np.array([x_value, y_value, z_value])
    #         transformed_point = self.tf_buffer.transform_point(point, 'camera_link')
    #         return (transformed_point.x, transformed_point.y, transformed_point.z)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         print(e)
    #         return (0, 0, 0)


    # def rospc_to_nppc(self, rospc):
    #     cloud_tuple = ros_numpy.numpify(rospc)
    #     cloud_tuple = ros_numpy.point_cloud2.split_rgb_field(cloud_tuple)

    #     # https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/
    #     cloud_array = np.zeros((self.img_dim_y,self.img_dim_x, 6), dtype=float)
    #     cloud_array[...,0] = cloud_tuple['x']
    #     cloud_array[...,1] = cloud_tuple['y']
    #     cloud_array[...,2] = cloud_tuple['z']

    #     cloud_array[...,3] = cloud_tuple['r']
    #     cloud_array[...,4] = cloud_tuple['g']
    #     cloud_array[...,5] = cloud_tuple['b']

    #     return cloud_array, cloud_tuple, rospc.header

    # def nppc_to_rospc(self, nppc, dtype, ros_msg_header):
    #     cloud_tuple = np.zeros_like(dtype)
    #     cloud_tuple['x'] = nppc[...,0]
    #     cloud_tuple['y'] = nppc[...,1]
    #     cloud_tuple['z'] = nppc[...,2]
    #     cloud_tuple['r'] = nppc[...,3]
    #     cloud_tuple['g'] = nppc[...,4]
    #     cloud_tuple['b'] = nppc[...,5]
    #     cloud_tuple = ros_numpy.point_cloud2.merge_rgb_fields(cloud_tuple)

    #     cloud_msg = ros_numpy.msgify(PointCloud2, cloud_tuple)
    #     cloud_msg.header = ros_msg_header # Only parts not inferrable from numpy array
    #     return cloud_msg

        

if __name__ == '__main__':
    try:
        camera = ImageInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass