#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import os
import re
import copy
from datetime import datetime
import time

import ransac_plane as pyrsc

import rospy
import rosbag
import cv2
#import open3d as o3d
#import pyransac3d as pyrsc
import numpy as np
import ros_numpy
import math
import tf2_ros as tf2
import message_filters
import tf2_geometry_msgs
import moveit_msgs.msg
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, MarkerArray
from std_msgs.msg import Empty
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from opencv_apps.msg import RotatedRectStamped, RotatedRect

#from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from scipy.spatial import KDTree
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from datetime import datetime
from tqdm import tqdm

np.set_printoptions(threshold=sys.maxsize)

# For real time debugging
class TimerError(Exception):

    """A custom exception used to report errors in use of Timer class"""

class Timer:
  def __init__(self):
    self._start_time = None

  def start(self):
    """Start a new timer"""
    if self._start_time is not None:
      raise TimerError("Timer is running. Use .stop() to stop it")

    self._start_time = time.perf_counter()

  def stop(self):
    """Stop the timer, and report the elapsed time"""
    if self._start_time is None:
      raise TimerError("Timer is not running. Use .start() to start it")

    elapsed_time = time.perf_counter() - self._start_time
    self._start_time = None
    #print(f"Elapsed time: {elapsed_time:0.4f} seconds")
    return elapsed_time


def rectifyImage_withReturn(self, raw):
  """
  :param raw:       input image
  :type raw:        :class:`CvMat` or :class:`IplImage`
  :param rectified: rectified output image
  :type rectified:  :class:`CvMat` or :class:`IplImage`

  Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
  """

  self.mapx = np.ndarray(shape=(self.height, self.width, 1),
                      dtype='float32')
  self.mapy = np.ndarray(shape=(self.height, self.width, 1),
                      dtype='float32')
  cv2.initUndistortRectifyMap(self.K, self.D, self.R, self.P,
          (self.width, self.height), cv2.CV_32FC1, self.mapx, self.mapy)
  return cv2.remap(raw, self.mapx, self.mapy, cv2.INTER_CUBIC)


class image_converter:

  def __init__(self):
    ### Wait for realsense2_camera topic
    #rospy.wait_for_message("/camera/depth/color/points",Image,timeout=15)
    # Get plain image database for mask -­> in process TODO
    plain_data_directory = "/home/damp2404/catkin_ws/src/rcm_modulaire/pa_uv/launch/img_plain"
    
    os.chdir(plain_data_directory)
    img_plain_list = os.listdir(plain_data_directory)
    self.img_plain_array = []
    self.int_plain_array = []
    for img in img_plain_list:
      self.int_plain_array.append(int(re.findall(r'\d+', img)[0]))
      self.img_plain_array.append(cv2.imread(img, 2))

    # For data analysis
    base_directory = "/home/damp2404/Documents/git_rcm/rcm_poncage/pa_uv/test_data"
    test_name = "data_{0}" .format(datetime.now())[:-7]
    self.data_directory = os.path.join(base_directory, test_name)
    os.mkdir(self.data_directory)

    self.offline_mode = rospy.get_param("/sensor_used/offline")

    # IMAGE PIPELINE -------------------------------------------------------------------------------------------------------------
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callbackRGB, queue_size=10)
    #self.image_3d_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callbackDepth, queue_size=10)

    if rospy.get_param("/sensor_used/offline"):
      self.img_dim_x = 1280
      self.img_dim_y = 720
    else:
      self.img_dim_x = rospy.get_param("/camera/realsense2_camera/color_width")
      self.img_dim_y = rospy.get_param("/camera/realsense2_camera/color_height")


    # Camera calibration
    self.camera_model = PinholeCameraModel()
    self.camera_resolutionX = 1.0  # mm/pixel
    self.camera_resolutionY = 1.0  # mm/pixel
    self.info_image_3d = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=1)
    self.info_image_3d.registerCallback(self.callbackInfoCam)

    self.last_cv_image_rectified = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint8)

    self.image_pub_norm = rospy.Publisher("image_norm", Image, queue_size=10) 
    self.image_pub_equal = rospy.Publisher("image_equal", Image, queue_size=10) 
    self.image_pub_otsu = rospy.Publisher("image_otsu", Image, queue_size=10) 
    self.image_pub_med = rospy.Publisher("image_med", Image, queue_size=10) 
    self.image_pub_med_dilate = rospy.Publisher("image_med_dilate", Image, queue_size=10) 
    self.image_pub_bin = rospy.Publisher("image_bin", Image, queue_size=10)
    self.image_pub_segmented = rospy.Publisher("image_segmented", Image, queue_size=10) 
    ### Box publisher for traj planification --Philippe D'Amours
    self.rotated_rect_pub = rospy.Publisher("bounding_box", RotatedRect , queue_size=10)

    # POINTCLOUD PIPELINE --------------------------------------------------------------------------------------------------------
    ##self.pc_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callbackPC, queue_size=10) ### ORIGINAL DE SAM avec rs_camera.launch
    self.pc_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callbackPC, queue_size=10) ###oganised point cloud avec rgbd_launch.launch.xml


    self.pc_raw = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)

    self.point_cloud_pub_plane = rospy.Publisher("point_cloud_plane", Marker, queue_size=10)


    # TEXTOVERLAY ----------------------------------------------------------------------------------------------------------------
    self.points_info = rospy.Publisher("points_info", OverlayText, queue_size=10)
    self.overlay_msg = OverlayText()
    self.overlay_msg.width = 500
    self.overlay_msg.height = 500
    self.overlay_msg.text_size = 10
    self.overlay_msg.left = 10
    self.overlay_msg.top = 10
    self.overlay_msg.font = "Ubuntu Mono Regular"
    self.overlay_msg.bg_color.a = 0
    self.overlay_msg.fg_color.r = 25 / 255.0
    self.overlay_msg.fg_color.g = 1
    self.overlay_msg.fg_color.b = 1
    self.overlay_msg.fg_color.a = 1

   
    # RVIZ INTERFACE ----------------------------------------------------------------------------------------------------------------
    self.target_interact_server = InteractiveMarkerServer("target_interact")
    self.target_rect_array = []   # Makers array
    self.target_marker_array = [] # Makers array

    self.img_index = 0
    self.full_path = ""
    self.save_image = rospy.Subscriber("/consol_save_image", Empty, self.callbackSaveImage, queue_size=10)
    self.save_takes_in_bag = rospy.Subscriber("/consol_save_takes", Empty, self.callbackSaveDefaultTakeInBag, queue_size=10)
    self.run_algo = rospy.Subscriber("/consol_run_algo", Empty, self.callbackrunAlgoAndPublish, queue_size=10)  # Only usefull in offline mode

    
  def rospc_to_nppc(self, rospc):
    cloud_tuple = ros_numpy.numpify(rospc)
    cloud_tuple = ros_numpy.point_cloud2.split_rgb_field(cloud_tuple)

    # https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/
    cloud_array = np.zeros((self.img_dim_y,self.img_dim_x, 6), dtype=float)
    cloud_array[...,0] = cloud_tuple['x']
    cloud_array[...,1] = cloud_tuple['y']
    cloud_array[...,2] = cloud_tuple['z']

    cloud_array[...,3] = cloud_tuple['r']
    cloud_array[...,4] = cloud_tuple['g']
    cloud_array[...,5] = cloud_tuple['b']

    return cloud_array, cloud_tuple, rospc.header

  def nppc_to_rospc(self, nppc, dtype, ros_msg_header):
    cloud_tuple = np.zeros_like(dtype)
    cloud_tuple['x'] = nppc[...,0]
    cloud_tuple['y'] = nppc[...,1]
    cloud_tuple['z'] = nppc[...,2]
    cloud_tuple['r'] = nppc[...,3]
    cloud_tuple['g'] = nppc[...,4]
    cloud_tuple['b'] = nppc[...,5]
    cloud_tuple = ros_numpy.point_cloud2.merge_rgb_fields(cloud_tuple)

    cloud_msg = ros_numpy.msgify(PointCloud2, cloud_tuple)
    cloud_msg.header = ros_msg_header # Only parts not inferrable from numpy array
    return cloud_msg


  def callbackInfoCam(self, data):
    # Get a camera model object using image_geometry and the camera_info topic
    self.camera_model.fromCameraInfo(data)
    self.info_image_3d.sub.unregister() #Subscribe only once

  def callbackSaveImage(self, data):
    # Save rgb image in current test folder 
    os.chdir(self.data_directory)
    img_bank_array = os.listdir(self.data_directory)
    int_bank_array = []
    for img in img_bank_array:
      int_bank_array.append(int(re.findall(r'\d+', img)[0]))
    
    try:
      last_img_num = max(int_bank_array)
    except ValueError:
      last_img_num = 0
      
    cv2.imwrite("img_raw{0}.png" .format(last_img_num+1), cv2.cvtColor(self.last_cv_image_rectified.astype(np.uint8), cv2.COLOR_RGB2BGR))
    rospy.loginfo("Now {0} images in bank for the current test" .format(last_img_num+1))
  
  def callbackSaveDefaultTakeInBag(self, data):
    # Save current takes in a rosbag for later default use
    if not(self.offline_mode):
      try:
        os.chdir("/home/introlab/Documents/git_rcm/rcm_poncage/pa_uv/launch")
        bag_name = "default_take.bag"
        bag = rosbag.Bag(bag_name, 'w')

        bag.write("image_rectified", self.bridge.cv2_to_imgmsg(self.last_cv_image_rectified, "rgb8"))
        bag.write("pc_raw", self.nppc_to_rospc(self.pc_raw, self.pc_dtype, self.pc_header_raw))

      except Exception as e:
        bag.close()
        rospy.logerr("Something went wrong with the writing of the bag:")
        rospy.logerr(e)
      else:
        bag.close()
        rospy.loginfo("An image and a point cloud (ros_msg) has been saved in {0} for the current take" .format(bag_name))

    else:
      rospy.logerr("Offline mode, camera probably not even connected")

  def callbackRGB(self, data):
    # Update current buffer image
    if self.camera_model.height != None:
        try:
          last_cv_image_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
          self.last_cv_image_rectified = rectifyImage_withReturn(self.camera_model, last_cv_image_raw)
        except CvBridgeError as e:
          rospy.logerr(e)
    else:
      rospy.logwarn("Camera model not available. New image not rectified")
      try:
        self.last_cv_image_rectified = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
      except CvBridgeError as e:
        rospy.logerr(e)

    # runAlgoAndPublish()

  def callbackPC(self, data):
    self.pc_raw, self.pc_dtype, self.pc_header_raw = self.rospc_to_nppc(data)


  def drawBox(self, image_bin, img_og, resX, resY):
    # Only elder contour, from top to bottom
    contours, hierarchy = cv2.findContours(image_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    img_box = img_og.copy()

    D_sander_m = 225  # (mm) -> https://www.festoolcanada.com/products/sanding/drywall-sander/571935---planex-lhs-e-225-eq-usa#Functions
    area_sander = ((math.pi * D_sander_m**2) /4) / (resX*resY)   # pixels
    #print("Sander area: {0}" .format(area_sander))
    min_side_size = 60 / resX  # size of screw patch converted in pixels (TODO: validate)
    #print("Sander diameter: {0}" .format(min_side_size))
    dim_array = np.array([[0,0]])
    multi_points_array = np.array([[[0,0],[0,0],[0,0],[0,0]]])
    single_point_array = np.array([[0,0]])

    size = 0
    biggest_rect = None
    for i, c in enumerate(contours):
        #print("Rectangle #:", i, " = ", c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        
        
        
        # Rect side size
        dim1 = np.sqrt( ((box[0,0]-box[1,0])**2)+((box[0,1]-box[1,1])**2) )
        dim2 = np.sqrt( ((box[1,0]-box[2,0])**2)+((box[1,1]-box[2,1])**2) )
        
        if(dim1 > min_side_size and dim2 > min_side_size):      
            dim_array = np.append(dim_array, [[dim1,dim2]], axis=0)

            if (dim1 * dim2) > area_sander:
              img_box = cv2.drawContours(img_box,[box],0,(255,0,0),2)
              multi_points_array = np.append(multi_points_array, [[[box[0,0], box[0,1]], 
                                                                  [box[1,0], box[1,1]], 
                                                                  [box[2,0], box[2,1]], 
                                                                  [box[3,0], box[3,1]]]], axis=0)               
            else:
                img_box = cv2.circle(img_box, (int((box[0,0]+box[2,0])/2), int((box[0,1]+box[2,1])/2)), radius=25, color=(255,255,255), thickness=2)
                single_point_array = np.append(single_point_array, [[(box[0,0]+box[2,0])/2, 
                                                                    (box[0,1]+box[2,1])/2]], axis=0)
            ### Biggest sized box for trajectory --Philippe D'Amours
            if (dim1 * dim2) > size:
              biggest_rect = rect
              size = dim1 * dim2
            
    ### Send the biggest box to trajectory pipeline --Philippe D'Amours
    if biggest_rect is None:
      rospy.logwarn("No rectangle found in image")
    else:
      rotated_rect_ = RotatedRect()
      rotated_rect_.angle = biggest_rect[2]
      rotated_rect_.center.x = biggest_rect[0][0] 
      rotated_rect_.center.y = biggest_rect[0][1] 
      rotated_rect_.size.height = biggest_rect[1][1]
      rotated_rect_.size.width  = biggest_rect[1][0]
    
      self.rotated_rect_pub.publish(rotated_rect_)


    #Remove first all zeros value
    dim_array = np.delete(dim_array, 0, 0)   
    multi_points_array = np.delete(multi_points_array, 0, 0)
    single_point_array = np.delete(single_point_array, 0, 0)


    
    

    return img_box, dim_array, multi_points_array, single_point_array
  
  def point2Dto3DAndPublish(self,x,y):
    ### take 2d coordinate in pixel and publish pose in camera tf

    # freeze point cloud
    pc_full = self.pc_raw
    # remove intensity
    pc = pc_full[..., :3]

    depth = pc[x][y]
    pass



  def global_plane(self, pc, color):
    # Point cloud must only have a lenght of 3 in the last dimension
    plane = pyrsc.Plane()
    best_eq, best_inliers, perc_success, it_success = plane.fit(pc, 0.003, minPoints=int(pc.shape[0]*pc.shape[1]*0.4), maxIteration=100)  # Good compromise
    best_eq = np.array(best_eq)
    if best_eq.size == 0:  # no planes found
      best_eq = np.array([1,1,1,1])  # https://pypi.org/project/pyransac3d/
    elif(best_eq[3] > 0):
      best_eq = -best_eq

    n = best_eq[:3]
    n_norm = np.linalg.norm(n)
    n_normalized = n / n_norm
    unitZ = np.array([0,0,1])
    distance = best_eq[3] / n_norm
    center_point = -1 * distance * n_normalized  # Not centered with point cloud... TODO: better fix than middle pixel of the image

    orientation = np.cross(unitZ, n_normalized)
    orientation = np.append(orientation, np.dot(unitZ, n_normalized) + np.sqrt(np.linalg.norm(unitZ)**2 + n_norm**2))
    orientation_normalized = orientation / np.linalg.norm(orientation)

    marker = Marker()
    #marker.lifetime = rospy.Duration(100)
    marker.frame_locked = True
    marker.header.frame_id = "camera_R435i_color_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.type = 1  # Square shape
    marker.id = 0
    marker.scale.x = 1 # m
    marker.scale.y = 1 # m
    marker.scale.z = 0.1  # m
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.5
    #marker.pose.position.x = center_point[0]
    #marker.pose.position.y = center_point[1]
    #marker.pose.position.z = center_point[2]
    mid_point = pc[int(pc.shape[0]/2), int(pc.shape[1]/2), :]
    marker.pose.position.x = mid_point[0]
    marker.pose.position.y = mid_point[1]
    marker.pose.position.z = mid_point[2]
    marker.pose.orientation.x = orientation_normalized[0]
    marker.pose.orientation.y = orientation_normalized[1] 
    marker.pose.orientation.z = orientation_normalized[2]
    marker.pose.orientation.w = orientation_normalized[3]

    return marker, n_normalized, mid_point[2], perc_success, it_success

  def callbackrunAlgoAndPublish(self, data):
    self.runAlgoAndPublish()

  def runAlgoAndPublish(self):
    valid_takes = False

    if not self.offline_mode:
      if(self.last_cv_image_rectified.any() and self.pc_raw.any()):
        valid_takes = True
      else:
        rospy.logwarn("No data received yet, please wait...")

    else:
      rospy.logwarn("Offline mode, camera probably not even connected. Default take will be used")

      try:
        base_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_uv/launch"
        os.chdir(base_directory)
        bag_name = "default_take.bag"
        bag = rosbag.Bag(bag_name)

        self.last_cv_image_rectified = self.bridge.imgmsg_to_cv2(next(bag.read_messages(topics=["image_rectified"]))[1], desired_encoding='rgb8')

        ros_msg_pc_raw = next(bag.read_messages(topics=["pc_raw"]))[1]
        # Patch: https://github.com/eric-wieser/ros_numpy/issues/2
        ros_msg_pc_raw.__class__ = PointCloud2
        self.pc_raw, self.pc_dtype, self.pc_header_raw = self.rospc_to_nppc(ros_msg_pc_raw)       

      except Exception as e:
        bag.close()
        rospy.logerr("Something went wrong with the reading of the bag:")
        rospy.logerr(e)
      else:
        bag.close()
        valid_takes = True
        rospy.loginfo("Default bag successfuly read, algo is starting")

    if valid_takes:
      t = Timer()

      # PREPROCESSING --------------------------------------------------------------------------------------------------------------
      t.start()
      # [R,G,B]
      color_take = [0,0,1]

      # Freeze pc for this algo run
      pc_full = self.pc_raw
      
      # Remove intensity
      pc = pc_full[..., :3]  

      # Estimate x,y resolution for image algo in mm/px
      self.camera_resolutionX = round(1000 * abs(pc[0,-1,0] - pc[0,0,0]) / pc.shape[1], 2)
      self.camera_resolutionY = round(1000 * abs(pc[-1,0,1] - pc[0,0,1]) / pc.shape[0], 2)

      if self.camera_resolutionX < 0.1 or self.camera_resolutionY < 0.1:
        rospy.logwarn('Warning impossible resolution:')
        rospy.logwarn("x: {0}, y: {1}\n" .format(self.camera_resolutionX , self.camera_resolutionY)) 
        rospy.logwarn("arbitrary 1.0 - 1.0 mm is used instead")
        self.camera_resolutionX = 1
        self.camera_resolutionY = 1 
      #else:
        #rospy.loginfo("ResX (mm/px): {0}\n" .format(self.camera_resolutionX))
        #rospy.loginfo("ResY (mm/px): {0}\n" .format(self.camera_resolutionY))

      #rospy.loginfo("Preprocessing:")
      t_preprocessing = t.stop()

      # Plane fit --------------------------------------------------------------------------------------------------------------
      t.start()

      # References planes
      ref_camera_normal = np.array([0,0,1])  # Constant from normal point of view
      # Comment for tests (text overlay and publisher as well) -­> Slowest part...
      marker_Gplane_takeA, normal_Gplane_takeA, distance_Gplane_takeA, perc_success, it_success = self.global_plane(pc, color_take)  # TODO: debug

      #rospy.loginfo("Plane fit:")
      t_plane_fit = t.stop()

      # IMAGE PROCESSING -------------------------------------------------------------------------------------------------------------
      t.start()

      # Take only the relevent color channel (replace by PCA?)
      image_algo = self.last_cv_image_rectified[...,2].copy()

      # Normalization, clahe
      img_norm = cv2.normalize(image_algo, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

      clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(6, 6))
      img_equal = clahe.apply(img_norm)

      # Ostu
      _, img_otsu = cv2.threshold(img_equal, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

      # Blur
      img_med = cv2.medianBlur(img_otsu, 45)  # TODO: adjust

      kernel = np.ones((15, 15), np.uint8)  # TODO: adjust
      img_med_dilate = cv2.dilate(img_med, kernel, iterations=1)

      # Binarisation
      thresh_arbitrary = 250
      _, img_bin = cv2.threshold(img_med_dilate, thresh_arbitrary, 255, cv2.THRESH_BINARY)
    
      # Draw box
      self.image_segmented, dim_array, multi_points_array, single_point_array = self.drawBox(img_bin, img_norm, self.camera_resolutionX, self.camera_resolutionY)

      #rospy.loginfo("Algo proper:")
      t_algo = t.stop()

      # TEXTOVERLAY ----------------------------------------------------------------------------------------------------------------
      self.overlay_msg.text =  "TakeA found {0} rectangle and {1} point\n" .format(len(multi_points_array), len(single_point_array))
      self.overlay_msg.text += "ResX-Y (mm/px): {0}-{1}\n" .format(self.camera_resolutionX, self.camera_resolutionY)
      #self.overlay_msg.text += "Normal distance: {0}cm, Mask distance: {1}cm\n" .format(distance_Gplane_cm, self.int_plain_array[index])
      self.overlay_msg.text += "RANSAC algo: {0} % of the points in {1} iterations\n" .format(round(perc_success,2), it_success)
      self.overlay_msg.text += "Processing time: Pre:{0}, Plane:{1}, Algo:{2}\n" .format(round(t_preprocessing,4), round(t_plane_fit,4), round(t_algo,4))
      self.overlay_msg.text += "--------------------------------------\n"

      # If there are new rectangle values
      if multi_points_array.any():
        for i, rect in enumerate(multi_points_array):
          self.overlay_msg.text += "Rectangle #{0} (mm):\n {1}\n" .format(i, rect)
      self.overlay_msg.text += "--------------------------------------\n"

      # If there are new points values
      if single_point_array.any():
        for i, point in enumerate(single_point_array):
          self.overlay_msg.text += "Point #{0} (mm):\n {1}\n" .format(i, point)
      self.overlay_msg.text += "--------------------------------------\n"

      # PUBLISHER ------------------------------------------------------------------------------------------------------------------
      try:
        self.image_pub_norm.publish(self.bridge.cv2_to_imgmsg(img_norm, "mono8"))
        self.image_pub_equal.publish(self.bridge.cv2_to_imgmsg(img_equal, "mono8"))
        self.image_pub_otsu.publish(self.bridge.cv2_to_imgmsg(img_otsu, "mono8"))
        self.image_pub_med.publish(self.bridge.cv2_to_imgmsg(img_med, "mono8"))
        self.image_pub_med_dilate.publish(self.bridge.cv2_to_imgmsg(img_med_dilate, "mono8"))
        self.image_pub_bin.publish(self.bridge.cv2_to_imgmsg(img_bin, "mono8"))
        self.image_pub_segmented.publish(self.bridge.cv2_to_imgmsg(self.image_segmented, "mono8"))
        self.points_info.publish(self.overlay_msg)
        self.point_cloud_pub_plane.publish(marker_Gplane_takeA)
      except CvBridgeError as e:
        print(e)

def main(args):
  rospy.init_node('image_converter')
  offline_mode = rospy.get_param("/sensor_used/offline")

  if offline_mode:
    rospy.logwarn("Offline mode, algo will only be usable with bag values")
  else:
    # Expecting camera, wait and retry until the camera node is present
    while not(rospy.has_param("/camera/realsense2_camera/color_width")) or not(rospy.has_param("/camera/realsense2_camera/color_height")):
      rospy.logwarn("Camera node not created, algo node will not start yet...")
      time.sleep(10)

  ic = image_converter()
  rospy.loginfo("Algo node up and running!")

  # Calculate algo and publish results at fixted rate -> replaces: rospy.spin()
  r = rospy.Rate(5)  # TODO: param in launch from camera...

  while not rospy.is_shutdown():
    r.sleep()

    if not offline_mode:
      ic.runAlgoAndPublish()

if __name__ == '__main__':
    main(sys.argv)
