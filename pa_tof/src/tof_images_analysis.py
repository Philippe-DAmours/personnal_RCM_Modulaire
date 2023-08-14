#!/usr/bin/env python
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
import open3d as o3d
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
      raise TimerError(f"Timer is running. Use .stop() to stop it")

    self._start_time = time.perf_counter()

  def stop(self):
    """Stop the timer, and report the elapsed time"""
    if self._start_time is None:
      raise TimerError(f"Timer is not running. Use .start() to start it")

    elapsed_time = time.perf_counter() - self._start_time
    self._start_time = None
    #print(f"Elapsed time: {elapsed_time:0.4f} seconds")
    return elapsed_time


class image_converter:

  def __init__(self):

    # Get plain image database for mask
    plain_data_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/launch/img_plain/"
    os.chdir(plain_data_directory)
    img_plain_list = os.listdir(plain_data_directory)
    self.img_plain_array = []
    self.int_plain_array = []
    for img in img_plain_list:
      self.int_plain_array.append(int(re.findall(r'\d+', img)[0]))
      self.img_plain_array.append(cv2.imread(img, 2))

    # For data analysis
    base_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/test_data/"
    test_name = "data_{0}" .format(datetime.now())[:-7]
    self.data_directory = os.path.join(base_directory, test_name)
    os.mkdir(self.data_directory)

    self.wanted_rate = rospy.get_param("/helios_camera_node/node_rate")
    self.offline_mode = rospy.get_param("/sensor_used/offline")
    
    # IMAGE PIPELINE -------------------------------------------------------------------------------------------------------------
    self.image_intensity_sub = rospy.Subscriber("/helios_camera_node/intensity_helios2", Image, self.callbackIntensity, queue_size=2)
    self.bridge = CvBridge()

    if self.offline_mode:
      self.img_dim_x = 640
      self.img_dim_y = 480
    else:
      self.img_dim_x = rospy.get_param("/helios_camera_node/intensity_width")
      self.img_dim_y = rospy.get_param("/helios_camera_node/intensity_height")

    # Scaling only used for ransac plane approx
    img_border_size_x = int(self.img_dim_x * 0.40)
    img_border_size_y = int(self.img_dim_y * 0.40)
    self.img_min_x = img_border_size_x
    self.img_max_x = self.img_dim_x-img_border_size_x
    self.img_min_y = img_border_size_y
    self.img_max_y = self.img_dim_y-img_border_size_y

    # To discriminate box size
    self.camera_resolutionX = 1.0  # mm/pixel
    self.camera_resolutionY = 1.0  # mm/pixel

    self.image_raw = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)

    self.image_pub_fix = rospy.Publisher("image_fix", Image, queue_size=10) 
    self.image_pub_plain_blur = rospy.Publisher("image_plain_blur", Image, queue_size=10) 
    self.image_pub_masked = rospy.Publisher("image_masked", Image, queue_size=10) 
    self.image_pub_rosin = rospy.Publisher("image_rosin", Image, queue_size=10) 
    self.image_pub_rosin_med = rospy.Publisher("image_rosin_med", Image, queue_size=10) 
    self.image_pub_rosin_dilate = rospy.Publisher("image_rosin_dilate", Image, queue_size=10) 
    self.image_pub_rosin_bin = rospy.Publisher("image_rosin_bin", Image, queue_size=10) 
    self.image_pub_segmented = rospy.Publisher("image_segmented", Image, queue_size=10) 


    # POINTCLOUD PIPELINE --------------------------------------------------------------------------------------------------------
    self.points_sub = rospy.Subscriber("/helios_camera_node/depth_helios2", PointCloud2, self.callbackTOF, queue_size=2)

    self.pc_raw = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)

    self.point_cloud_pub_scaled = rospy.Publisher("point_cloud_scaled", PointCloud2, queue_size=10)
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
    self.img_index = 0
    self.full_path = ""
    self.save_image = rospy.Subscriber("/consol_save_image", Empty, self.callbackSaveImage, queue_size=10)
    self.save_default_take_in_bag = rospy.Subscriber("/consol_save_default_take_in_bag", Empty, self.callbackSaveDefaultTakeInBag, queue_size=10)

    self.run_algo = rospy.Subscriber("/consol_run_algo", Empty, self.callbackrunAlgoAndPublish, queue_size=10)  # Only usefull in offline mode

    
  def rospc_to_nppc(self, rospc):
    # x = 0, y = 1, z = 2, i = 3
    cloud_tuple = ros_numpy.numpify(rospc)
    dtype = cloud_tuple.dtype
    cloud_array = np.array([[[a,b,c,d] for a, b, c ,d in temp] for temp in cloud_tuple])
    #print(dtype)
    #print(cloud_array.shape)
    #print(cloud_array[320,240])
    return cloud_array, dtype, rospc.header

  def nppc_to_rospc(self, nppc, dtype, ros_msg_header):
    # x = 0, y = 1, z = 2, i = 3
    cloud_tuple = np.array([[(a,b,c,d) for a, b, c ,d in temp] for temp in nppc], dtype=dtype)
    #print(cloud_tuple.shape)
    #print(cloud_tuple[320,240])
    cloud_msg = ros_numpy.msgify(PointCloud2, cloud_tuple)
    cloud_msg.header = ros_msg_header # Only parts not inferrable from numpy array + dtype
    return cloud_msg


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
      
    cv2.imwrite("img_raw{0}.png" .format(last_img_num+1), self.image_raw.astype(np.uint16))
    rospy.loginfo("Now {0} images in bank for the current test" .format(last_img_num+1)) 

  def callbackSaveDefaultTakeInBag(self, data):
    # Save current takes in a rosbag for later default use
    if not(self.offline_mode):
      try:
        os.chdir("/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/launch")
        bag_name = "default_take.bag"
        bag = rosbag.Bag(bag_name, 'w')

        bag.write("image_raw", self.bridge.cv2_to_imgmsg(self.image_raw, "mono16"))
        bag.write("pc_raw", self.nppc_to_rospc(self.pc_raw, self.pc_dtype_raw, self.pc_header_raw))

      except Exception as e:
        bag.close()
        rospy.logerr("Something went wrong with the writing of the bag:")
        rospy.logerr(e)
      else:
        bag.close()
        rospy.loginfo("An image and a point cloud (ros_msg) has been saved in {0} for the current take" .format(bag_name))

    else:
      rospy.logerr("Offline mode, camera probably not even connected")


  def callbackIntensity(self, data):
    # Update current buffer image
    try:
      self.image_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono16')
    except CvBridgeError as e:
      print(e)

  def callbackTOF(self, data):
    self.pc_raw, self.pc_dtype_raw, self.pc_header_raw = self.rospc_to_nppc(data)


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
    #print(len(contours))

    for i, c in enumerate(contours):
        #print("Rectangle #:", i, " = ", c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        #print(box.dtype)
        #print(((box[0,0]+box[2,0])/2).dtype)
        
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

    #Remove first all zeros value
    dim_array = np.delete(dim_array, 0, 0)   
    multi_points_array = np.delete(multi_points_array, 0, 0)
    single_point_array = np.delete(single_point_array, 0, 0)

    return img_box, dim_array, multi_points_array, single_point_array

  def rosinThreshold(self, image):
    hist,bins = np.histogram(image.ravel(), (np.amax(image) - np.amin(image)))
    nbr_bin = hist.size
    
    # left most peak replace: peak_x = hist.argmax()
    delta_index = 5
    delta = 100
    peak_x = 0
    for vx, vy in reversed(list(enumerate(hist))):
        if vx < nbr_bin-delta_index:
            if (hist[vx+delta_index]+delta < hist[vx]) and (hist[vx-delta_index]+delta < hist[vx]):
                peak_x = vx
                break
    
    #last_x = np.argwhere(hist>0)[-1,0]
    last_x = np.argwhere(hist>int(peak_x/1000))[-1,0]    # 0.1% de la valeur  peak (arbitraire)

    # maximises the perpendicular distance
    p1 = np.array([peak_x, hist[peak_x]])
    p2 = np.array([last_x, hist[last_x]])

    d = np.zeros_like(hist)  # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    for vx, vy in enumerate(hist):
        if vx > peak_x and vx < last_x:
            pv = np.array([vx, vy])
            d[vx] = np.abs(np.cross(p2-p1, p1-pv)) / np.linalg.norm((p2-p1))

    # Prendre le milieu en cas de multiple valeurs identiques
    # threshold_rosin = d.argmax()
    indexes = np.argwhere(d==d.max()).flatten()
    threshold_rosin = indexes[int(indexes.shape[0]/2)-1]

    ret, image_rosin_thresh = cv2.threshold(image, threshold_rosin, 255, cv2.THRESH_TOZERO)
    return image_rosin_thresh

  def global_plane(self, pc, color):
    # Point cloud must only have a lenght of 3 in the last dimension
    plane = pyrsc.Plane()
    best_eq, best_inliers, perc_success, it_success = plane.fit(pc, 0.002, minPoints=int(pc.shape[0]*pc.shape[1]*0.4), maxIteration=50)  # Good compromise
    best_eq = np.array(best_eq)
    if(best_eq[3] > 0):
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
    marker.header.frame_id = "camera_frame"
    marker.header.stamp = rospy.Time.now()
    marker.type = 1  # Square shape
    marker.id = 0
    marker.scale.x = 1 # m
    marker.scale.y = 1 # m
    marker.scale.z = 0.001  # m
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
    #rospy.loginfo("Algo iteration -------------------------------------------------------")

    if not self.offline_mode:
      if(self.image_raw.any() and self.pc_raw.any()):
        valid_takes = True
        #rospy.loginfo("Current data will be use for the following calculation")
      else:
        rospy.logwarn("No data received yet, please wait...")

    else:
      rospy.logwarn("Offline mode, camera probably not even connected. Default take will be used")

      try:
        base_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/launch"
        os.chdir(base_directory)
        bag_name = "default_take.bag"
        bag = rosbag.Bag(bag_name)

        self.image_raw = self.bridge.imgmsg_to_cv2(next(bag.read_messages(topics=["image_raw"]))[1], desired_encoding='mono16')

        ros_msg_pc_raw = next(bag.read_messages(topics=["pc_raw"]))[1]
        # Patch: https://github.com/eric-wieser/ros_numpy/issues/2
        ros_msg_pc_raw.__class__ = PointCloud2
        self.pc_raw, self.pc_dtype_raw, self.pc_header_raw = self.rospc_to_nppc(ros_msg_pc_raw)       

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
      color_take = [0,0,1]  # [R,G,B]

      # Cut the outer/biast part of the image (scaling)
      scaled_pc_full = self.pc_raw[self.img_min_y:self.img_max_y, self.img_min_x:self.img_max_x, :]
      
      # Remove intensity
      scaled_pc = scaled_pc_full[..., :3]  

      # Estimate x,y resolution for image algo in mm/px
      self.camera_resolutionX = round(1000 * abs(scaled_pc[0,-1,0] - scaled_pc[0,0,0]) / scaled_pc.shape[1], 2)
      self.camera_resolutionY = round(1000 * abs(scaled_pc[-1,0,1] - scaled_pc[0,0,1]) / scaled_pc.shape[0], 2)

      if self.camera_resolutionX < 0.1 or self.camera_resolutionY < 0.1:
        rospy.logwarn('Warning impossible resolution:')
        rospy.logwarn("x: {0}, y: {1}\n" .format(self.camera_resolutionX , self.camera_resolutionY)) 
        rospy.logwarn("arbitrary 0.1 - 0.1 mm is used instead")
        self.camera_resolutionX = 0.1
        self.camera_resolutionY = 0.1 
      #else:
        #rospy.loginfo("ResX (mm/px): {0}\n" .format(self.camera_resolutionX))
        #rospy.loginfo("ResY (mm/px): {0}\n" .format(self.camera_resolutionY))

      #rospy.loginfo("Preprocessing:")
      t_preprocessing = t.stop()

      # Plane fit --------------------------------------------------------------------------------------------------------------
      t.start()

      # References planes
      ref_camera_normal = np.array([0,0,1])  # Constant from normal point of view
      marker_Gplane, normal_Gplane, distance_Gplane, perc_success, it_success = self.global_plane(scaled_pc, color_take)

      #rospy.loginfo("Plane fit:")
      t_plane_fit = t.stop()

      # IMAGE PROCESSING -------------------------------------------------------------------------------------------------------------
      t.start()

      # fix algo image
      image_algo = self.image_raw.copy()
      img_min = np.amin(image_algo)
      img_max = np.amax(image_algo)
      
      # Backgrouund selection, closest to live normal distance
      distance_Gplane_cm = round(distance_Gplane*100,2)
      index = np.abs(self.int_plain_array-distance_Gplane_cm).argmin()
      img_plain = self.img_plain_array[index].copy()

      # Mask
      img_plain_blur = cv2.blur(img_plain, (51,51))
      img_plain_blur_norm = cv2.normalize(img_plain_blur, None, img_min, img_max, cv2.NORM_MINMAX, dtype=cv2.CV_16U)
      img_masked = cv2.absdiff(image_algo, img_plain_blur_norm)

      # Bilateral - edge preserving
      img_dst4 = cv2.edgePreservingFilter(img_masked, flags=1, sigma_s=60, sigma_r=0.4)  # Faster
      #img_dst4 = cv2.bilateralFilter(img_masked.astype(np.float32), 60, 60, 60)  # Good but very slow -> try sobel?
      #img_dst4_norm = cv2.normalize(img_dst4, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)  # Float to int for bilateral
      img_rosin = self.rosinThreshold(img_dst4)

      # 8bit normalization
      img_rosin_norm = cv2.normalize(img_rosin, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U).copy()

      # Blur
      img_rosin_med = cv2.medianBlur(img_rosin_norm, 21)  # TODO: adjust

      kernel = np.ones((15, 15), np.uint8)  # TODO: adjust
      img_rosin_dilate = cv2.dilate(img_rosin_med, kernel, iterations=1)

      # Binarisation
      thresh_arbitrary = 200
      _, img_rosin_bin = cv2.threshold(img_rosin_dilate, thresh_arbitrary, 255, cv2.THRESH_BINARY)
    
      # Draw box
      image_segmented, dim_array, multi_points_array, single_point_array = self.drawBox(img_rosin_bin, img_rosin_norm, self.camera_resolutionX, self.camera_resolutionY)

      #rospy.loginfo("Algo proper:")
      t_algo = t.stop()

      # TEXTOVERLAY ----------------------------------------------------------------------------------------------------------------
      self.overlay_msg.text =  "TakeA found {0} rectangle and {1} point\n" .format(len(multi_points_array), len(single_point_array))
      self.overlay_msg.text += "Res X-Y (mm/px): {0} - {1}\n" .format(self.camera_resolutionX, self.camera_resolutionY)
      self.overlay_msg.text += "Normal distance: {0}cm, Mask distance: {1}cm\n" .format(distance_Gplane_cm, self.int_plain_array[index])
      self.overlay_msg.text += "RANSAC algo: {0} % of the points in {1} iterations\n" .format(round(perc_success,2), it_success)
      self.overlay_msg.text += "Processing time: Pre:{0}, Plane:{1}, Algo:{2}\n" .format(round(t_preprocessing,4), round(t_plane_fit,4), round(t_algo,4))
      self.overlay_msg.text += "---------------------------------------------------------\n"

      # If there are new rectangle values
      if multi_points_array.any():
        for i, rect in enumerate(multi_points_array):
          self.overlay_msg.text += "Rectangle #{0} (mm):\n {1}\n" .format(i, rect)
      self.overlay_msg.text += "---------------------------------------------------------\n"

      # If there are new points values
      if single_point_array.any():
        for i, point in enumerate(single_point_array):
          self.overlay_msg.text += "Point #{0} (mm):\n {1}\n" .format(i, point)
      self.overlay_msg.text += "---------------------------------------------------------\n"

      # PUBLISHER ------------------------------------------------------------------------------------------------------------------
      ros_msg_pc_raw = self.nppc_to_rospc(scaled_pc_full, self.pc_dtype_raw, self.pc_header_raw)
      try:
        self.image_pub_fix.publish(self.bridge.cv2_to_imgmsg(image_algo, "mono16"))
        self.image_pub_plain_blur.publish(self.bridge.cv2_to_imgmsg(img_plain_blur_norm, "mono16"))
        self.image_pub_masked.publish(self.bridge.cv2_to_imgmsg(img_masked, "mono16"))
        self.image_pub_rosin.publish(self.bridge.cv2_to_imgmsg(img_rosin_norm, "mono8"))
        self.image_pub_rosin_med.publish(self.bridge.cv2_to_imgmsg(img_rosin_med, "mono8"))
        self.image_pub_rosin_dilate.publish(self.bridge.cv2_to_imgmsg(img_rosin_dilate, "mono8"))
        self.image_pub_rosin_bin.publish(self.bridge.cv2_to_imgmsg(img_rosin_bin, "mono8"))
        self.image_pub_segmented.publish(self.bridge.cv2_to_imgmsg(image_segmented, "mono8"))
        self.point_cloud_pub_scaled.publish(ros_msg_pc_raw)
        self.points_info.publish(self.overlay_msg)
        self.point_cloud_pub_plane.publish(marker_Gplane)
      except CvBridgeError as e:
        print(e)
      

def main(args):
  rospy.init_node('image_converter')
  offline_mode = rospy.get_param("/sensor_used/offline")

  if offline_mode:
    rospy.logwarn("Offline mode, algo will only be usable with bag values")
  else:
    # Expecting camera, wait and retry until the camera node is present
    while not(rospy.has_param("/helios_camera_node/intensity_width")) or not(rospy.has_param("/helios_camera_node/intensity_height")):
      rospy.logwarn("Camera node not created, algo node will not start yet...")
      time.sleep(10)

  ic = image_converter()
  rospy.loginfo("Algo node up and running!")

  # Calculate algo and publish results at fixted rate -> replaces: rospy.spin()
  r = rospy.Rate(ic.wanted_rate)

  while not rospy.is_shutdown():
    r.sleep()

    if not offline_mode:
      ic.runAlgoAndPublish()

if __name__ == '__main__':
    main(sys.argv)