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
    print(f"Elapsed time: {elapsed_time:0.4f} seconds")


class image_converter:

  def __init__(self):

    # For data analysis
    base_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/test_data/"
    test_name = "data_{0}" .format(datetime.now())[:-7]
    self.data_directory = os.path.join(base_directory, test_name)
    os.mkdir(self.data_directory)

    # IMAGE PIPELINE -------------------------------------------------------------------------------------------------------------
    self.image_intensity_sub = rospy.Subscriber("/helios_camera_node/intensity_helios2", Image, self.callbackIntensity, queue_size=10)
    self.image_points_sub = rospy.Subscriber("/helios_camera_node/depth_helios2", PointCloud2, self.callbackTOF, queue_size=10)

    self.bridge = CvBridge()

    self.wanted_rate = rospy.get_param("/helios_camera_node/node_rate")
    if rospy.get_param("/sensor_used/offline"):
      self.img_dim_x = 640
      self.img_dim_y = 480
    else:
      self.img_dim_x = rospy.get_param("/helios_camera_node/intensity_width")
      self.img_dim_y = rospy.get_param("/helios_camera_node/intensity_height")
    # Scale larger on takeB to make shure that all of takeA is included
    self.border_size_x_takeA = int(self.img_dim_x * 0.30)
    self.border_size_y_takeA = int(self.img_dim_y * 0.30)
    self.border_size_x_takeB = int(self.img_dim_x * 0.20)
    self.border_size_y_takeB = int(self.img_dim_y * 0.20)

    self.last_cv_image_raw = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)
    self.image_takeA = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)
    self.image_takeB = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)
    self.image_algoA = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)
    self.image_algoB = np.zeros((self.img_dim_y,self.img_dim_x,3), dtype=np.uint16)
    self.image_segmented = np.zeros((self.img_dim_y-2*self.border_size_y_takeA,self.img_dim_x-2*self.border_size_x_takeA,3), dtype=np.uint16)

    self.image_pub_textureA = rospy.Publisher("image_textureA", Image, queue_size=10)
    self.image_pub_textureB = rospy.Publisher("image_textureB", Image, queue_size=10)   
    self.image_pub_segmented = rospy.Publisher("image_segmented", Image, queue_size=10) 


    # POINTCLOUD PIPELINE --------------------------------------------------------------------------------------------------------
    self.last_pc_msg = PointCloud2()
    self.pc_takeA = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)
    self.pc_takeB = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)
    self.dtype_takeA = np.dtype
    self.dtype_takeB = np.dtype
    self.header_takeA = PointCloud2().header
    self.header_takeB = PointCloud2().header

    self.Ro = np.ones((self.img_dim_y-2*self.border_size_y_takeA, self.img_dim_x-2*self.border_size_x_takeA), dtype=float)*0.95
    self.n = np.ones_like(self.Ro)*1.15
    self.new_param = False

    self.point_cloud_pub_takeA = rospy.Publisher("point_cloud_takeA", PointCloud2, queue_size=10)
    self.point_cloud_pub_takeB = rospy.Publisher("point_cloud_takeB", PointCloud2, queue_size=10)
    self.point_cloud_pub_takeB_inprogress = rospy.Publisher("point_cloud_takeB_inprogress", PointCloud2, queue_size=10)
    self.point_cloud_pub_takeB_warpped = rospy.Publisher("point_cloud_takeB_warpped", PointCloud2, queue_size=10)

    self.point_cloud_pub_normalA = rospy.Publisher("point_cloud_normalA", MarkerArray, queue_size=10)
    self.point_cloud_pub_normalB = rospy.Publisher("point_cloud_normalB", MarkerArray, queue_size=10)

    self.point_cloud_pub_planeA = rospy.Publisher("point_cloud_planeA", Marker, queue_size=10)
    self.point_cloud_pub_planeB = rospy.Publisher("point_cloud_planeB", Marker, queue_size=10)


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
    self.save_takes_in_bag = rospy.Subscriber("/consol_save_takes", Empty, self.callbackSaveTakeInBag, queue_size=10)

    self.takenA = False
    self.takenB = False
    self.take_image_A = rospy.Subscriber("/consol_take_image_A", Empty, self.callbackTakeImageA, queue_size=10)
    self.take_image_B = rospy.Subscriber("/consol_take_image_B", Empty, self.callbackTakeImageB, queue_size=10)
    self.run_algo = rospy.Subscriber("/consol_run_algo", Empty, self.runAlgoAndPublish, queue_size=10)

    
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
      
    cv2.imwrite("img_raw{0}.png" .format(last_img_num+1), self.last_cv_image_raw.astype(np.uint16))
    rospy.loginfo("Now {0} images in bank for the current test" .format(last_img_num+1))
  
  def saveTakeInBag(self, base_directory):
    try:
      os.chdir(base_directory)
      bag_name = "default_takes.bag"
      bag = rosbag.Bag(bag_name, 'w')

      bag.write("image_takeA", self.bridge.cv2_to_imgmsg(self.image_takeA, "mono16"))
      bag.write("image_takeB", self.bridge.cv2_to_imgmsg(self.image_takeB, "mono16"))
      bag.write("pc_takeA", self.nppc_to_rospc(self.pc_takeA, self.dtype_takeA, self.header_takeA))
      bag.write("pc_takeB", self.nppc_to_rospc(self.pc_takeB, self.dtype_takeB, self.header_takeB))  

    except Exception as e:
      bag.close()
      rospy.logerr("Something went wrong with the writing of the bag:")
      rospy.logerr(e)
    else:
      bag.close()
      rospy.loginfo("An image and a point cloud (ros_msg) has been saved in {0} for both takes" .format(bag_name))

  def callbackSaveTakeInBag(self, data):
    # Save current takes in a rosbag for later default use

    # Run only if both take are aquired
    if(self.takenA and self.takenB):
      self.saveTakeInBag("/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/launch")
    else:
      rospy.logerr("Data incomplet, at least one take missing")


  def callbackTakeImageA(self, data):
    rospy.loginfo("Current image saved as A")
    self.image_takeA = self.last_cv_image_raw
    self.pc_takeA, self.dtype_takeA, self.header_takeA = self.rospc_to_nppc(self.last_pc_msg)
    self.takenA = True

  def callbackTakeImageB(self, data):
    rospy.loginfo("Current image saved as B")
    self.image_takeB = self.last_cv_image_raw
    self.pc_takeB, self.dtype_takeB, self.header_takeB = self.rospc_to_nppc(self.last_pc_msg)
    self.takenB = True


  def callbackIntensity(self, data):
    # Update current buffer image
    try:
      self.last_cv_image_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono16')
    except CvBridgeError as e:
      print(e)

  def callbackTOF(self, data):
    self.last_pc_msg = data
   

  def incidence_angle(self, point_cloud, scaled_pc, border_size_x, border_size_y, color):
    # Point cloud must only have a lenght of 3 in the last dimension
    cam_pose = np.array([0,0,0])  # Angle computed from normal point of view
    v = scaled_pc - cam_pose
    v = v/np.linalg.norm(v, axis=2)[..., None]
    v_full = point_cloud[..., :3] - cam_pose
    n = np.zeros_like(v)

    k = 11  #TODO: adjust according to the rest of the algo
    k_half = int((k-1)/2)  # Has to be smaller than border_size_x AND border_size_y
    kernel_size = (k,k)
    # Compute on full pc, but only inner part is used
    v_centroid = cv2.blur(v_full, kernel_size, cv2.BORDER_REPLICATE)
    v_centroid = v_centroid[border_size_y:self.img_dim_y-border_size_y, border_size_x:self.img_dim_x-border_size_x, :]

    # For display purposes
    normal_marker_list = []
    normal_marker_index = 0

    # Inspired from: https://github.com/PickNikRobotics/rviz_visual_tools/blob/master/src/rviz_visual_tools.cpp
    # and: https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    for i in tqdm(range(v.shape[0]), desc ="Angle of incidence: ", leave=True):
      for j in range(v.shape[1]):
        v_neighbors = v_full[(i+border_size_y-k_half-1):(i+border_size_y+k_half), (j+border_size_x-k_half-1):(j+border_size_x+k_half), :].reshape(k**2, 3)

        u, s, vh = np.linalg.svd(v_neighbors - v_centroid[i, j], full_matrices=False, compute_uv=True)
        if vh[-1,2] < 0:
          n[i,j] = vh[-1,:]
        else:
          n[i,j] = vh[-1,:] * -1

        # For display in rviz
        if not normal_marker_index%200:
          marker = Marker()
          marker.header.frame_id = "camera_frame"
          marker.header.stamp = rospy.Time.now()
          marker.type = 0  # Arrow shape
          marker.id = normal_marker_index
          marker.scale.x = 0.002  # m
          marker.scale.y = 0.006  # m
          marker.scale.z = 0.0
          marker.color.r = color[0]
          marker.color.g = color[1]
          marker.color.b = color[2]
          marker.color.a = 0.75
          marker.pose.orientation.x = 0.0  # For warning...
          marker.pose.orientation.y = 0.0  # For warning...
          marker.pose.orientation.z = 0.0  # For warning...
          marker.pose.orientation.w = 1.0  # For warning...
          P_start = Point()
          P_start.x = scaled_pc[i,j,0]
          P_start.y = scaled_pc[i,j,1]
          P_start.z = scaled_pc[i,j,2]
          P_end = Point()
          P_end.x = scaled_pc[i,j,0] + (n[i,j,0]/10)
          P_end.y = scaled_pc[i,j,1] + (n[i,j,1]/10)
          P_end.z = scaled_pc[i,j,2] + (n[i,j,2]/10)
          marker.points = [P_start, P_end]
          normal_marker_list.append(marker)

        normal_marker_index += 1
    
    product = np.einsum('ijk,ijk->ij', (v * -1), n)  # Inverse v and DOT product with n
    theta = np.arccos(product)

    return theta, normal_marker_list


  def global_plane(self, pc, color):
    # Point cloud must only have a lenght of 3 in the last dimension
    plane = pyrsc.Plane()
    best_eq, best_inliers = plane.fit(pc, 0.0015, minPoints=int(pc.shape[0]*pc.shape[1]*0.4), maxIteration=10000)  # Good compromise
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

    return marker, n_normalized, mid_point[2]  #distance


  def residual(self, param, rA, thetaA, rB, thetaB):
    Ro = param[0]
    n = param[1]
    return (Ro*(np.cos((2-1/n)*thetaA))**n - rA) + (Ro*(np.cos((2-1/n)*thetaB))**n - rB)
  
  def residual_single(self, param, r, theta):
    Ro = param[0]
    n = param[1]
    return Ro*(np.cos((2-1/n)*theta))**n - r

  def runAlgoAndPublish(self, data):
    valid_takes = False

    if(self.takenA and self.takenB):
      valid_takes = True
      self.saveTakeInBag(self.data_directory)
      rospy.loginfo("Data complet, both takes saved in {0} as a ros bag\n They will be use for the following calculation" .format(self.data_directory))

    else:
      rospy.logwarn("Data incomplet, default previously saved takes will be used")

      try:
        base_directory = "/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/launch"
        os.chdir(base_directory)
        bag_name = "default_takes.bag"
        bag = rosbag.Bag(bag_name)

        self.image_takeA = self.bridge.imgmsg_to_cv2(next(bag.read_messages(topics=["image_takeA"]))[1], desired_encoding='mono16')
        self.image_takeB = self.bridge.imgmsg_to_cv2(next(bag.read_messages(topics=["image_takeB"]))[1], desired_encoding='mono16')

        ros_msg_pc_takeA = next(bag.read_messages(topics=["pc_takeA"]))[1]
        ros_msg_pc_takeB = next(bag.read_messages(topics=["pc_takeB"]))[1]
        # Patch: https://github.com/eric-wieser/ros_numpy/issues/2
        ros_msg_pc_takeA.__class__ = PointCloud2
        ros_msg_pc_takeB.__class__ = PointCloud2
        self.pc_takeA, self.dtype_takeA, self.header_takeA = self.rospc_to_nppc(ros_msg_pc_takeA)       
        self.pc_takeB, self.dtype_takeB, self.header_takeB = self.rospc_to_nppc(ros_msg_pc_takeB)

      except Exception as e:
        bag.close()
        rospy.logerr("Something went wrong with the reading of the bag:")
        rospy.logerr(e)
      else:
        bag.close()
        valid_takes = True
        rospy.loginfo("Default bag successfuly read for both takes, algo is starting")

    if valid_takes:
      t = Timer()

      # PREPROCESSING --------------------------------------------------------------------------------------------------------------
      t.start()
      # [R,G,B]
      color_takeA = [0,0,1]
      color_takeB = [1,0,1]

      # To validate algo with "perfect" data TODO
      synt_data = rospy.get_param("/sensor_used/synt_data")
      if synt_data:
        rospy.logwarn("synthetic data parameter will be used")
        dummy_pcA = np.zeros_like(self.pc_takeA)
        dummy_pcB = np.zeros_like(self.pc_takeB)

        dummy_pcA[..., 0] = np.tile(np.linspace(-0.4, 0.4, dummy_pcA.shape[1]), (dummy_pcA.shape[0],1))
        dummy_pcA[..., 1] = np.tile(np.linspace(0.25, -0.25, dummy_pcA.shape[0])[None,...].T, (1,dummy_pcA.shape[1]))
        dummy_pcA[..., 2] = 0.75

        print("Dummy_pcA shape and samples points:")
        print(dummy_pcA.shape)
        print(dummy_pcA[0,0])
        print(dummy_pcA[100,100])
        print(dummy_pcA[-1,-1])

        dummy_pcB[..., 0] = np.tile(np.linspace(-0.5, 0.5, dummy_pcB.shape[1]), (dummy_pcB.shape[0],1))
        dummy_pcB[..., 2] = np.tile(np.linspace(1, 0.5, dummy_pcB.shape[1]), (dummy_pcB.shape[0],1))
        y_lim = np.linspace(0.4, 0.2, dummy_pcB.shape[1])
        for i, _ in enumerate(dummy_pcB[...,1].T):
          dummy_pcB[:,i,1] = np.linspace(y_lim[i], -y_lim[i], dummy_pcB.shape[0])

        print("Dummy_pcB shape and samples points:")
        print(dummy_pcB.shape)
        print(dummy_pcB[0,0])
        print(dummy_pcB[100,100])
        print(dummy_pcB[-1,-1])

        self.pc_takeA = dummy_pcA
        self.pc_takeB = dummy_pcB

      # Cut the outer/biast part of the image (scaling)
      scaled_pcA_full = self.pc_takeA[self.border_size_y_takeA:self.img_dim_y-self.border_size_y_takeA, self.border_size_x_takeA:self.img_dim_x-self.border_size_x_takeA, :]
      scaled_pcB_full = self.pc_takeB[self.border_size_y_takeB:self.img_dim_y-self.border_size_y_takeB, self.border_size_x_takeB:self.img_dim_x-self.border_size_x_takeB, :]

      # Remove intensity
      scaled_pcA = scaled_pcA_full[..., :3]  
      scaled_pcB = scaled_pcB_full[..., :3]  

      rospy.loginfo("Preprocessing:")
      t.stop()

      # ANGLE OF INCIDENCE --------------------------------------------------------------------------------------------------------    
      t.start()
      # Compute incidences angles
      thetaA, normal_marker_listA = self.incidence_angle(self.pc_takeA, scaled_pcA, self.border_size_x_takeA, self.border_size_y_takeA, color_takeA)
      thetaB_temp, normal_marker_listB = self.incidence_angle(self.pc_takeB, scaled_pcB, self.border_size_x_takeB, self.border_size_y_takeB, color_takeB)

      rospy.loginfo("Angle of incidence:")
      t.stop()

      # WARPING --------------------------------------------------------------------------------------------------------------
      t.start()

      # References planes
      ref_camera_normal = np.array([0,0,1])  # Constant from normal point of view
      marker_Gplane_takeA, normal_Gplane_takeA, distance_Gplane_takeA = self.global_plane(scaled_pcA, color_takeA)
      marker_Gplane_takeB, normal_Gplane_takeB, distance_Gplane_takeB = self.global_plane(scaled_pcB, color_takeB)

      # Geometric relation
      Gplane_thetaA = np.arccos(ref_camera_normal@normal_Gplane_takeA)
      Gplane_thetaB = np.arccos(ref_camera_normal@normal_Gplane_takeB)
      # To be independent of the side of takeB
      if Gplane_thetaA < Gplane_thetaB:
        Gplane_delta_theta = Gplane_thetaA-Gplane_thetaB
        dir_x = -1
      else:
        Gplane_delta_theta = Gplane_thetaB-Gplane_thetaA
        dir_x = 1
      Gplane_quaternion = quaternion_from_euler(0,Gplane_delta_theta,0)

      Gplane_hypo = 2 * distance_Gplane_takeA * np.cos((np.pi/2)-(Gplane_delta_theta/2))  # distance_Gplane_takeA sould = distance_Gplane_takeB

      # Transformation matrix -­> TODO: get transformation from robot TFs
      t_msg = TransformStamped()
      t_msg.header.frame_id = "angle_frame"
      t_msg.header.stamp = rospy.Time.now()
      t_msg.child_frame_id = "camera_frame"
      t_msg.transform.translation.x = dir_x * Gplane_hypo *  np.cos(Gplane_delta_theta/2)
      t_msg.transform.translation.y = 0.0
      t_msg.transform.translation.z = Gplane_hypo *  np.sin(Gplane_delta_theta/2)
      t_msg.transform.rotation.x = Gplane_quaternion[0]
      t_msg.transform.rotation.y = Gplane_quaternion[1]
      t_msg.transform.rotation.z = Gplane_quaternion[2]
      t_msg.transform.rotation.w = Gplane_quaternion[3]      

      transform_matrix = ros_numpy.numpify(t_msg.transform)

      # Warp pcB
      homo_axe = np.ones((scaled_pcB.shape[0], scaled_pcB.shape[1], 1))
      scaled_pcB_homo = np.concatenate((scaled_pcB,homo_axe), axis=2)
      scaled_pcB_warp = (scaled_pcB_homo@transform_matrix.T)[..., :-1]  # Transform and remove homogeneous part
      scaled_pcB_warp_inprogress = np.concatenate((scaled_pcB_warp, scaled_pcB_full[...,3][...,None]), axis=2)  # Add back intensity for display purpose

      # Instanciation (thetas were computed in the previous section)
      iA = scaled_pcA_full[..., 3]
      dA = scaled_pcA_full[..., 2]

      iB = np.zeros_like(scaled_pcA_full[..., 3])
      dB = np.zeros_like(scaled_pcA_full[..., 2])
      thetaB = np.zeros_like(thetaA)

      # KDtree
      nbr_interp = 3
      nbr_represent_optimal = int(np.sqrt(scaled_pcB_warp.shape[0]*scaled_pcB_warp.shape[1]))
      tree = KDTree(scaled_pcB_warp.reshape((-1,3), order='F'), leafsize=nbr_represent_optimal, copy_data=True)  
      #ind = tree.query(scaled_pcA.reshape(-1, 3), k=1, return_distance=False)  # If we want to remove the for loop

      # Apply warp to intensity, distance and angle of takeB
      for i, points_list in enumerate(tqdm(scaled_pcA, desc ="Warping: ", leave=True)):
        for j, pointA in enumerate(points_list):
          # Eucledian distance, only with x and y
          #closest = np.unravel_index(np.argmin(np.sum(np.abs(scaled_pcB_warp[...,:-1] - pointA[:-1]), axis=2)), scaled_pcB_warp.shape[:-1])  # https://stackoverflow.com/questions/48135736/what-is-an-intuitive-explanation-of-np-unravel-index

          # KDtree, with x, y, z
          dd, ii = tree.query(pointA[None,...], k=nbr_interp, distance_upper_bound=0.1, workers=8)  # distance_upper_bound max 10cm
          closest = np.unravel_index(ii, scaled_pcB_warp.shape[:-1], order='F')

          # from tuple to usable array
          closest_array = np.concatenate((closest[0], closest[1]), axis=0)  

          # Interpolate
          nbr_neighbors_found = closest_array.shape[1]
          for l in range(nbr_neighbors_found):
            iB[i,j] = iB[i,j] + scaled_pcB_full[closest_array[0,l],closest_array[1,l],3]/nbr_neighbors_found
            dB[i,j] = dB[i,j] + scaled_pcB_full[closest_array[0,l],closest_array[1,l],2]/nbr_neighbors_found
            thetaB[i,j] = thetaB[i,j] + thetaB_temp[closest_array[0,l],closest_array[1,l]]/nbr_neighbors_found

      # Update x, y, z to fit with takeA and add intensity for display
      #scaled_pcB_warp[..., 0:2] = scaled_pcA[..., 0:2]  # if only x, y
      scaled_pcB_warp_full = np.concatenate((scaled_pcA, iB[..., None]), axis=2)

      rospy.loginfo("Warping:")
      t.stop()

      # ALGO PROPER --------------------------------------------------------------------------------------------------------    
      t.start()

      k = 10  # Radiometric scale factor [(R(spectralon)/R(mesure))/0.6]
      coeff = [8.94, 3.92, 1.79, 1.38, 4.44]  # A, B, C, D, E from values in article
      fA = (coeff[0] + coeff[1]/dA**2) * (coeff[2]*thetaA**2 + coeff[3]*thetaA + coeff[4]) # arbitrary physical model from article
      fB = (coeff[0] + coeff[1]/dB**2) * (coeff[2]*thetaB**2 + coeff[3]*thetaB + coeff[4]) # arbitrary physical model from article
      if synt_data:
        synt_Ro = 0.85
        synt_n = 1.15
        rA = synt_Ro*np.power(np.cos((2-1/synt_n)*thetaA), synt_n)
        rB = synt_Ro*np.power(np.cos((2-1/synt_n)*thetaB), synt_n)
      else:
        rA = iA / (fA*k)  # Absolute surface reflectance
        rB = iB / (fB*k)  # Absolute surface reflectance

      # Define bounds for parameters
      bounds = ([0.5, 1.0], [1.5, 1.3])
      
      # Define least squares problem for each pixel in the image
      for i in tqdm(range(rA.shape[0]), desc ="Least square adjustment: ", leave=True):
        for j in tqdm(range(rA.shape[1]), desc ="current row: ", leave=False):

          # Define initial parameter guesses
          params = [self.Ro[i,j], self.n[i,j]]
          result = least_squares(self.residual, params, bounds=bounds, args=(rA[i,j], thetaA[i,j], rB[i,j], thetaB[i,j]), ftol=0.001, xtol=0.001, gtol=0.001, verbose=0)
          #result = least_squares(self.residual_single, params, bounds=bounds, args=(rB[i,j], thetaB[i,j]), xtol=0.001, verbose=0)
      
          # Extract estimated parameters
          self.Ro[i,j] = result.x[0]
          self.n[i,j] = result.x[1]
          if not result.success:
           print("pixel nbr:", i, ",", j, "failed to converge")

          # Print estimated parameters
          self.new_param = True
      
      '''# Estimate Ro and n from R_i and theta_i
      deltaC = np.ones_like(np.stack((thetaA,thetaB), axis=2))[..., :, None]
      deltaC_filtered = np.ones_like(deltaC)
      e = np.ones_like(deltaC)
      
      tolerance = 0.01
      while abs(np.mean(e)) > tolerance:
        print("Iteration start -----------------------------------------------------------------------------")
        Ro = self.Ro
        n = self.n
        test = np.cos((2-1/n)*thetaA)
        print("cos -> ", "min:", np.amin(test), "max:", np.amax(test), "mean:", np.mean(test), "std:", np.std(test), "var:", np.var(test))
        print("Ro -> ", "min:", np.amin(Ro), "max:", np.amax(Ro), "mean:", np.mean(Ro), "std:", np.std(Ro), "var:", np.var(Ro))
        print("n -> ", "min:", np.amin(n), "max:", np.amax(n), "mean:", np.mean(n), "std:", np.std(n), "var:", np.var(n))
        dRA_dRo = np.power(np.cos((2-1/n)*thetaA), n)
        dRA_dn = Ro * np.power(np.cos((2-1/n)*thetaA), n) * (np.log(np.cos((2-1/n)*thetaA)) - np.tan(2-1/n)/n)
        dRB_dRo = np.power(np.cos((2-1/n)*thetaB), n)
        dRB_dn = Ro * np.power(np.cos((2-1/n)*thetaB), n) * (np.log(np.cos((2-1/n)*thetaB)) - np.tan(2-1/n)/n)
        a = np.transpose(np.array([[dRA_dRo, dRA_dn], [dRB_dRo, dRB_dn]]), (2,3,0,1))
        #a_T = np.transpose(a, (0,1,3,2))
        e = np.transpose(np.array([[rA - Ro*np.power(np.cos((2-1/n)*thetaA), n)], [rB - Ro*np.power(np.cos((2-1/n)*thetaB), n)]]), (2,3,0,1))
        print("A: ", a.shape)
        #print("A_T: ", a_T.shape)
        print("E: ", e.shape)
        print("deltaC: ", deltaC.shape)

        #deltaC = np.linalg.inv(a_T@a) @ a_T @ e  # for non-square "a" matrix
        deltaC = np.linalg.inv(a) @ e
        print("E -> ", "min:", np.amin(e), "max:", np.amax(e), "mean:", np.mean(e), "std:", np.std(e), "var:", np.var(e))
        print("deltaC -> ", "min:", np.amin(deltaC), "max:", np.amax(deltaC), "mean:", np.mean(deltaC), "std:", np.std(deltaC), "var:", np.var(deltaC))

        learning_rate = 0.0001  #TODO: adjust
        deltaC_filtered = np.where(np.abs(e)>tolerance, deltaC, 0*deltaC)*learning_rate
        deltaC_simple_filtered = np.squeeze(deltaC_filtered, axis=-1)
        self.Ro = Ro + deltaC_simple_filtered[...,0]
        self.n = n + deltaC_simple_filtered[...,1]
        self.n = np.clip(self.n, a_min=0, a_max=None)
        self.new_param = True'''

      # Filter parameters to help generalize -> doesn't help...
      '''k = 7  # arbitrary values from article
      kernel_size = (k,k)
      self.Ro = cv2.blur(self.Ro, kernel_size, cv2.BORDER_REPLICATE)
      self.n = cv2.blur(self.n, kernel_size, cv2.BORDER_REPLICATE)
      self.new_param = True'''

      rospy.loginfo("Algo proper:")
      t.stop()

      # IMAGE PROCESSING -------------------------------------------------------------------------------------------------------------
      t.start()

      # Identify the used part of the image
      self.image_algoA = cv2.rectangle(self.image_takeA, (self.border_size_x_takeA, self.border_size_y_takeA), (self.img_dim_x-self.border_size_x_takeA, self.img_dim_y-self.border_size_y_takeA), (0,0,255), 3) 
      self.image_algoB = cv2.rectangle(self.image_takeB, (self.border_size_x_takeB, self.border_size_y_takeB), (self.img_dim_x-self.border_size_x_takeB, self.img_dim_y-self.border_size_y_takeB), (0,0,255), 3) 

      # Display segmentation results
      #self.image_segmented = np.where((self.Ro>=9.5) & (self.n<1.25), 255, 0).astype(self.image_algoA.dtype)
      self.image_segmented = ((self.n - 1) * 65535).astype(self.image_algoA.dtype)

      rospy.loginfo("Image:")
      t.stop()

      # TEXTOVERLAY ----------------------------------------------------------------------------------------------------------------
      thetaA_deg = np.rad2deg(thetaA)
      thetaB_deg = np.rad2deg(thetaB)
      self.overlay_msg.text =  "TakeA:\n"
      self.overlay_msg.text += "General angle: {0}\n" .format(np.rad2deg(Gplane_thetaA))
      self.overlay_msg.text += "Specific moy: {0}\n" .format(np.mean(thetaA_deg))
      self.overlay_msg.text += "Specific deviation: {0}\n\n" .format(np.std(thetaA_deg))
      self.overlay_msg.text += "TakeB:\n"
      self.overlay_msg.text += "General: {0}\n" .format(np.rad2deg(Gplane_thetaB))
      self.overlay_msg.text += "Specific moy: {0}\n" .format(np.mean(thetaB_deg))
      self.overlay_msg.text += "Specific deviation: {0}\n" .format(np.std(thetaB_deg))
      self.overlay_msg.text += "debug---------\n"
      self.overlay_msg.text += "General delta: {0}\n" .format(np.rad2deg(Gplane_delta_theta))

      self.overlay_msg.text += "--------------------------------------\n"

      # PUBLISHER ------------------------------------------------------------------------------------------------------------------
      ros_msg_pc_takeA = self.nppc_to_rospc(scaled_pcA_full, self.dtype_takeA, self.header_takeA)
      ros_msg_pc_takeB = self.nppc_to_rospc(scaled_pcB_full, self.dtype_takeB, self.header_takeB)
      ros_msg_pc_takeB_inprogress = self.nppc_to_rospc(scaled_pcB_warp_inprogress, self.dtype_takeB, self.header_takeB)
      ros_msg_pc_takeB_warpped = self.nppc_to_rospc(scaled_pcB_warp_full, self.dtype_takeB, self.header_takeB)
      try:
        self.image_pub_textureA.publish(self.bridge.cv2_to_imgmsg(self.image_algoA, "mono16"))
        self.image_pub_textureB.publish(self.bridge.cv2_to_imgmsg(self.image_algoB, "mono16"))
        self.image_pub_segmented.publish(self.bridge.cv2_to_imgmsg(self.image_segmented, "mono16"))
        self.point_cloud_pub_takeA.publish(ros_msg_pc_takeA)
        self.point_cloud_pub_takeB.publish(ros_msg_pc_takeB)
        self.point_cloud_pub_takeB_inprogress.publish(ros_msg_pc_takeB_inprogress)
        self.point_cloud_pub_takeB_warpped.publish(ros_msg_pc_takeB_warpped)
        self.points_info.publish(self.overlay_msg)
        self.point_cloud_pub_normalA.publish(normal_marker_listA)
        self.point_cloud_pub_normalB.publish(normal_marker_listB)
        self.point_cloud_pub_planeA.publish(marker_Gplane_takeA)
        self.point_cloud_pub_planeB.publish(marker_Gplane_takeB)
      except CvBridgeError as e:
        print(e)
      

def main(args):
  rospy.init_node('image_converter')

  if rospy.get_param("/sensor_used/offline"):
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

  # Display parm graph
  plt.ion()
  animated_plot = plt.plot(ic.n.flatten(), ic.Ro.flatten(), marker='o', linestyle='none')[0]
  plt.title("Surface physic parameters")
  plt.xlabel("n")
  plt.ylabel("Ro")
  plt.axis([1, 1.3, 0.5, 1.5])
  plt.draw()

  while not rospy.is_shutdown():
    r.sleep()

    if (ic.new_param):
      animated_plot.set_xdata(ic.n.flatten())
      animated_plot.set_ydata(ic.Ro.flatten())
      plt.draw()
      plt.pause(0.1)
      ic.new_param = False

    #ic.runAlgoAndPublish()
  
  # Save param and close plot
  np.save(os.path.join(ic.data_directory,  "param_Ro"), ic.Ro)
  np.save(os.path.join(ic.data_directory,  "param_n"), ic.n)

  rospy.loginfo("Both param array saved succesfully in {0}" .format(ic.data_directory))
  plt.close('all')

if __name__ == '__main__':
    main(sys.argv)