#!/usr/bin/env python
from __future__ import print_function

import sys
import os
import re
import copy
from datetime import datetime

import moveit_commander

import rospy
import cv2
import numpy as np
import math
import tf2_ros as tf2
import message_filters
import tf2_geometry_msgs
import moveit_msgs.msg
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, MarkerArray
from std_msgs.msg import Empty
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel


def rosinThreshold(image):
  hist,bins = np.histogram(image.ravel(),256,[0,256])
  peak_x = hist.argmax()
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

def drawBox(image, resX, resY):
  # Only elder contour, from top to bottom
  _, contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  img_box = image.copy()

  D_sander_m = 225  # (mm) -> https://www.festoolcanada.com/products/sanding/drywall-sander/571935---planex-lhs-e-225-eq-usa#Functions
  area_sander = ((math.pi * D_sander_m**2) /4) / (resX*resY)   # pixels
  #print("Sander area: {0}" .format(area_sander))
  min_side_size = 80 / resX  # size of screw patch converted in pixels (TODO: validate)
  #print("nbr box {0}" .format(i))  print("nbr box {0}" .format(i))#print("Sander diameter: {0}" .format(min_side_size))
  dim_array = np.array([[0,0]])
  multi_points_array = np.array([[[0,0],[0,0],[0,0],[0,0]]])
  single_point_array = np.array([[0,0]])
  #print(len(contours))

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
              img_box = cv2.circle(img_box, ((box[0,0]+box[2,0])/2, (box[0,1]+box[2,1])/2), radius=12, color=(255,255,255), thickness=2)
              single_point_array = np.append(single_point_array, [[(box[0,0]+box[2,0])/2, 
                                                                  (box[0,1]+box[2,1])/2]], axis=0)  

  dim_array = np.delete(dim_array, 0, 0)   #Remove first all zeros value
  multi_points_array = np.delete(multi_points_array, 0, 0)   #Remove first all zeros value
  single_point_array = np.delete(single_point_array, 0, 0)   #Remove first all zeros value
  # print("Dimension de chaques rectangles: \n", dim_array)
  # print("4 points contours de chaques grands rectangles: \n", multi_points_array)
  # print("Point central de chaques petits rectangles: \n", single_point_array)

  return img_box, dim_array, multi_points_array, single_point_array


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

def within_image(p, max_x, max_y):
  min_x = 0
  min_y = 0

  if p[0] < min_y:
    p[0] = min_y

  elif p[1] < min_x:
    p[1] = min_x

  elif p[0] > max_y:
    p[0] = max_y

  elif p[1] > max_x:
    p[1] = max_x

  else:
    pass

def depth_moy(k_size, center_point, depth_img):
  sum = 0
  start_pointX = center_point[0] - (center_point[0]-1)/2
  start_pointY = center_point[1] - (center_point[1]-1)/2
  for i, _ in enumerate(range(k_size), start=start_pointX):
    for j, _ in enumerate(range(k_size), start=start_pointY):
      depth_live = depth_img[j, i]
      sum = sum+depth_live

  return sum/(k_size**2)


def get_robot_transform(source_frame, target_frame, parent_id):
  tf_buffer = tf2.Buffer(rospy.Duration(2.0))
  tf2.TransformListener(tf_buffer)

  try:
    trans = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(), rospy.Duration(0.5))
    trans.header.seq = parent_id + 1   # Duplicate to keep the first pose identity
  except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException, tf2.InvalidArgumentException) as e:
    print(e)

  return trans

def do_robot_transform(pose_source, source_frame, target_frame):
  transformation = get_robot_transform(source_frame, target_frame, pose_source.header.seq)

  return tf2_geometry_msgs.do_transform_pose(pose_source, transformation)

def create_cam_frame_pose(point, frame, id_num):
  # Convert from mm to m, adjust axis to fit the camera's tf and cast in standard msg
  poseS = PoseStamped()
  poseS.header.seq = id_num
  poseS.header.stamp = rospy.Time.now()
  poseS.header.frame_id = frame
  poseS.pose.position.x = point[2]/1000
  poseS.pose.position.y = -point[0]/1000
  poseS.pose.position.z = -point[1]/1000
  poseS.pose.orientation.x = 0.0
  poseS.pose.orientation.y = 0.0
  poseS.pose.orientation.z = 0.0
  poseS.pose.orientation.w = 1.0
  
  return poseS

def robot_pose_2_marker(pose, color):
  marker = Marker()

  marker.header.frame_id = pose.header.frame_id
  marker.header.stamp = rospy.Time.now()

  # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
  marker.type = Marker.CUBE
  marker.id = pose.header.seq
  marker.action = Marker.ADD

  # Set the scale of the marker
  if marker.type == Marker.ARROW:
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01
  else:
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

  # Set the color
  marker.color.r = color[0]
  marker.color.g = color[1]
  marker.color.b = color[2]
  marker.color.a = color[3]

  # Set the pose of the marker
  marker.pose = pose.pose

  # Set lifetime for display purposes
  marker.lifetime = rospy.Duration(0.1)   # 1 frames @ 6fps

  return marker

def marker_array_2_interactive_marker(marker_array_list, frame, name_list, desc):
  control_array = []   # Interactive marker control array
  inter_marker = InteractiveMarker()   # Proper/complet interactive marker

  inter_marker.header.seq = 0
  inter_marker.header.stamp = rospy.Time.now()
  inter_marker.header.frame_id = frame

  # Center of the interactive markers "array"
  inter_marker.pose.position.x = 0.0
  inter_marker.pose.position.y = 0.0
  inter_marker.pose.position.z = 0.0
  inter_marker.pose.orientation.x = 0.0
  inter_marker.pose.orientation.y = 0.0
  inter_marker.pose.orientation.z = 0.0
  inter_marker.pose.orientation.w = 1.0

  # Must be unique
  inter_marker.name = ("target_from_{0}_{1}" .format(name_list[0], name_list[1]))

  # Must be smaller than 40 characters
  inter_marker.description = desc

  # Default value
  inter_marker.scale = 1

  # Both markers arrays
  for i, marker_array in enumerate(marker_array_list):
    control_array.append(InteractiveMarkerControl())

    control_array[i].name = ("{0}_p" .format(name_list[i]))
    control_array[i].interaction_mode = InteractiveMarkerControl.BUTTON
    control_array[i].markers = marker_array
    control_array[i].always_visible = True

  inter_marker.controls = control_array

  return inter_marker


class image_converter:

  def __init__(self):
    # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    # reference^^
    self.image_pub_gray = rospy.Publisher("image_gray", Image, queue_size=10)
    self.image_pub_clahe = rospy.Publisher("image_clahe", Image, queue_size=10)
    self.image_pub_rosin = rospy.Publisher("image_rosin", Image, queue_size=10)
    self.image_pub_2ponce = rospy.Publisher("image_2ponce", Image, queue_size=10)
    self.image_pub_rectified = rospy.Publisher("image_rectified", Image, queue_size=10)
    self.image_pub_filtered_depth = rospy.Publisher("image_filtered_depth", Image, queue_size=10)

    img_res_x = rospy.get_param("/camera/realsense2_camera/color_width")
    img_res_y = rospy.get_param("/camera/realsense2_camera/color_height")
    self.last_cv_image_raw = np.zeros((img_res_y,img_res_x,3), dtype=np.uint8)
    self.last_cv_image_rectified = np.zeros((img_res_y,img_res_x,3), dtype=np.uint8)
    self.last_cv_image_gray_filtered = np.zeros((img_res_y,img_res_x), dtype=np.float32)
    self.last_cv_image_gray = np.zeros((img_res_y,img_res_x), dtype=np.uint8)
    self.last_cv_image_clahe = np.zeros((img_res_y,img_res_x), dtype=np.uint8)
    self.last_cv_image_rosin = np.zeros((img_res_y,img_res_x), dtype=np.uint8)
    self.last_cv_image_2ponce = np.zeros((img_res_y,img_res_x), dtype=np.uint8)
    self.last_cv_depth_image_raw = np.zeros((img_res_y,img_res_x), dtype=np.float32)
    self.last_cv_depth_image_filtered = np.zeros((img_res_y,img_res_x), dtype=np.float32)

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
    

    self.runmean_lenght = 12  # 2sec at 6fps
    self.rect_runmean_i = 0
    self.rect_max = 6
    self.rect_runmean_array = np.zeros([self.runmean_lenght, self.rect_max, 4, 2])
    self.rect_runmean_mean = np.zeros([self.rect_max, 4, 2])
    self.rect_runmean_var = np.zeros([self.rect_max, 4, 2])
    self.rect_runmean_std = np.zeros([self.rect_max])
    self.point_runmean_i = 0
    self.point_max = 4*9
    self.point_runmean_array = np.zeros([self.runmean_lenght, self.point_max, 2])
    self.point_runmean_mean = np.zeros([self.point_max, 2])
    self.point_runmean_var = np.zeros([self.point_max, 2])
    self.point_runmean_std = np.zeros([self.point_max])

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callbackRGB, queue_size=10)

    self.camera_model = PinholeCameraModel()
    self.camera_resolutionX = 1.0  # mm/pixel
    self.camera_resolutionY = 1.0  # mm/pixel
    self.info_image_3d = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=1)
    self.info_image_3d.registerCallback(self.callbackInfoCam)
    self.image_3d_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callbackDepth, queue_size=10)
    self.target_rect_pub = rospy.Publisher("target_rect", MarkerArray, queue_size=10)     # Unused -> line:509
    self.target_point_pub = rospy.Publisher("target_point", MarkerArray, queue_size=10)   # Unused -> line:513
    self.target_interact_server = InteractiveMarkerServer("target_interact")
    self.target_rect_array = []   # Makers array
    self.target_marker_array = [] # Makers array       

    self.save_image = rospy.Subscriber("/consol_save_image", Empty, self.callbackSaveImage, queue_size=10)
    self.go = rospy.Subscriber("/consol_go", Empty, self.callbackRobotGo, queue_size=10)
    self.img_index = 0
    self.full_path = ""
    self.chosen_target = PoseStamped()

    # Check if the robot is activated in the launch file
    self.robot_connected = rospy.get_param("/sensor_used/robot")
    if self.robot_connected:
      self.moveit_plan = moveit_msgs.msg.RobotTrajectory()   # Empty plan
      try:
        # Reference: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        self.robot = moveit_commander.RobotCommander()
        self.moveit_commander = moveit_commander.MoveGroupCommander("manipulator", ns="/", wait_for_servers=20)
        print("Succesfully conntected to {0} controler" .format(self.moveit_commander.get_name()))
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
      except Exception as e:
        print(e)

  def interactive_marker_feedback(self, feedback):
    # Reference: http://docs.ros.org/en/api/visualization_msgs/html/msg/InteractiveMarker.html
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
      poseS = PoseStamped()
      poseS.header = feedback.header
      poseS.pose.position = feedback.mouse_point
      poseS.pose.orientation = feedback.pose.orientation
      # Make shure the mouse is in the right reference frame
      self.chosen_target = do_robot_transform(poseS, "base_link", feedback.header.frame_id)

      print("1)-------------------------------")
      print(feedback)
      if self.robot_connected:
        print("2)-------------------------------")
        print(self.chosen_target)
        try:
          waypoints = []
          wpose = self.moveit_commander.get_current_pose().pose
          waypoints.append(copy.deepcopy(wpose))
          # Don't change orientation -> TODO: adapt with wall plane
          wpose.position = self.chosen_target.pose.position
          wpose.position.x = self.chosen_target.pose.position.x + 0.15   # TODO: remove offset if the urdf is up to date
          waypoints.append(copy.deepcopy(wpose))

          # TODO: Smoother planner
          (self.moveit_plan, fraction) = self.moveit_commander.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

          display_trajectory = moveit_msgs.msg.DisplayTrajectory()
          display_trajectory.trajectory_start = self.robot.get_current_state()
          display_trajectory.trajectory.append(self.moveit_plan)
          self.display_trajectory_publisher.publish(display_trajectory)
        except Exception as e:
          print(e)

  def callbackRobotGo(self, data):
    print("CAUTION: The robot should move!")
    self.moveit_commander.execute(self.moveit_plan, wait=True)
    self.moveit_commander.stop()   # For safety
      
  def callbackSaveImage(self, data):
    # Save rgb image in databank
    base_directory = "/home/introlab/Documents/git_RCM/rcm_poncage/pa_rgb/images/"
    databank_directory = os.path.join(base_directory, "databank_img_rgb")
    os.chdir(databank_directory)
    img_bank_array = os.listdir(databank_directory)
    int_bank_array = []
    for img in img_bank_array:
      int_bank_array.append(int(re.findall(r'\d+', img)[0]))
    
    try:
      last_img_num = max(int_bank_array)
    except ValueError:
      last_img_num = 0
    
    cv2.imwrite("img_rect{0}.png" .format(last_img_num+1), cv2.cvtColor(self.last_cv_image_rectified, cv2.COLOR_BGR2RGB))
    print("-------------------------------")
    print("Now {0} images in bank" .format(last_img_num+1))

    # Create new folder for detailled data save -> only the first callback
    if self.img_index == 0:
      now = datetime.now()
      now_string = now.strftime("%d_%m_%Y_at_%H_%M")
      folder = "img_" + now_string
      self.full_path = os.path.join(base_directory, folder)
      try:
        os.mkdir(self.full_path)
      except OSError as error: 
        print(error)

    # Save detailled data in timestamped folder
    os.chdir(self.full_path)
    # Add other if needs be
    filename_list = ["img_2ponce{0}.png" .format(self.img_index), "img_rect{0}.png" .format(self.img_index), "img_fdep{0}.png" .format(self.img_index)]    
    image_tosave_list = [self.last_cv_image_2ponce, cv2.cvtColor(self.last_cv_image_rectified, cv2.COLOR_BGR2RGB), self.last_cv_depth_image_filtered.astype(np.uint16)]

    for i, file in enumerate(filename_list):
      cv2.imwrite(file, image_tosave_list[i])

    print("{0} image saved for the {1} time this run at {2}\n" .format(i+1, self.img_index+1, self.full_path))
    self.img_index += 1

  def callbackInfoCam(self, data):
    # Get a camera model object using image_geometry and the camera_info topic
    self.camera_model.fromCameraInfo(data)
    self.info_image_3d.sub.unregister() #Subscribe only once

  def callbackDepth(self, data):
    # https://github.com/IntelRealSense/realsense-ros/issues/714
    try:
      self.last_cv_depth_image_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
      cv2.accumulateWeighted(self.last_cv_depth_image_raw, self.last_cv_depth_image_filtered, 0.1)
    except CvBridgeError as e:
      print(e)

    depth_img_use = self.last_cv_depth_image_raw
    # Make shure that the camera model is loaded and that the depth image is not empty
    if self.camera_model.height != None and depth_img_use.any() == True:
      img_centerX = self.camera_model.width/2
      img_centerY = self.camera_model.height/2
      kernel_size = 3
      dt_px = kernel_size-1

      p1x_index = np.array([img_centerX-dt_px,img_centerY-dt_px])
      p2x_index = np.array([img_centerX+dt_px,img_centerY+dt_px])
      p1_depth = depth_moy(kernel_size, p1x_index, depth_img_use)
      p2_depth = depth_moy(kernel_size, p2x_index, depth_img_use)

      p1_m = [(num*p1_depth) for num in self.camera_model.projectPixelTo3dRay(p1x_index)]      
      p2_m = [(num*p2_depth) for num in self.camera_model.projectPixelTo3dRay(p2x_index)]

      # mm/px
      self.camera_resolutionX = round(abs(p1_m[0] - p2_m[0]) / (2*dt_px), 1)
      self.camera_resolutionY = round(abs(p1_m[1] - p2_m[1]) / (2*dt_px), 1)

      if self.camera_resolutionX < 0.1 or self.camera_resolutionY < 0.1:
        print('Warning both points are to close:')
        print("p1_m: {0}, p2_m: {1}\n" .format(p1_m, p2_m)) 
        print("arbitrary 0.5 - 0.5 mm is used instead")
        self.camera_resolutionX = 0.5
        self.camera_resolutionY = 0.5   

    try:
      self.image_pub_filtered_depth.publish(self.bridge.cv2_to_imgmsg(self.last_cv_depth_image_filtered.astype(np.uint16), "mono16"))
    except CvBridgeError as e:
      print(e)

  def callbackRGB(self, data):
    # Make shure that the camera model is loaded
    if self.camera_model.height != None:
      try:
        self.last_cv_image_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
        self.last_cv_image_rectified = rectifyImage_withReturn(self.camera_model, self.last_cv_image_raw)
      except CvBridgeError as e:
        print(e)

    self.last_cv_image_gray = cv2.cvtColor(self.last_cv_image_rectified, cv2.COLOR_BGR2GRAY)
    cv2.accumulateWeighted(self.last_cv_image_gray, self.last_cv_image_gray_filtered, alpha=0.1)
    clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(8, 8))
    self.last_cv_image_clahe = clahe.apply(self.last_cv_image_gray_filtered.astype(np.uint8))
    self.last_cv_image_rosin = rosinThreshold(self.last_cv_image_clahe)
    self.last_cv_image_2ponce, dim_array, multi_points_array, single_point_array = drawBox(self.last_cv_image_rosin, self.camera_resolutionX, self.camera_resolutionY)

    self.overlay_msg.text = "Resolution (mm): {0} - {1}\n" .format(self.camera_resolutionX, self.camera_resolutionY)
    self.overlay_msg.text += "{0} rectangles where found\n" .format(len(multi_points_array))
    self.overlay_msg.text += "{0} points where found\n" .format(len(single_point_array))
    self.overlay_msg.text += "--------------------------------------\n"

    # Set/reset unique id for markers
    marker_id = 0

    # Depth image used for the 3D transformations
    depth_img_use = self.last_cv_depth_image_filtered

    # If there are new rectangle values
    if multi_points_array.any():
      # Remove overflow rectangles
      if len(multi_points_array) > self.rect_max:
        multi_points_array = np.delete(multi_points_array, np.s_[self.rect_max:], 0)

      for i, rect in enumerate(multi_points_array):
        self.rect_runmean_array[self.rect_runmean_i,i,:,:] = rect
        self.rect_runmean_mean[i,:,:] = np.mean(self.rect_runmean_array[:,i,:,:], 0)
        self.rect_runmean_var[i,:,:] = np.var(self.rect_runmean_array[:,i,:,:], 0)
        self.rect_runmean_std[i] = round(np.sqrt(np.sum(self.rect_runmean_var[i,:,:])/(self.rect_runmean_var[i,:,:].size)), 2)   #https://stats.stackexchange.com/questions/25848/how-to-sum-a-standard-deviation

        self.overlay_msg.text += "Rectangle #{0} (mm) [std:{1}]:\n" .format(i, self.rect_runmean_std[i])
        for point in rect:
          within_image(point, depth_img_use.shape[0], depth_img_use.shape[1])  # patch because rect can be outside of the image 
          depth = depth_img_use[(point[1]-1), (point[0]-1)]
          # self.overlay_msg.text += "- Depth:{0}" .format(depth)
          point_3D = [round(num*depth) for num in self.camera_model.projectPixelTo3dRay(point)]
          self.overlay_msg.text += "{1}\n" .format(i, point_3D)
          pose_3D = create_cam_frame_pose(point_3D, "map", marker_id)
          pose_3D_transformed = do_robot_transform(pose_3D, "base_link", "camera_R435i_link")
          marker_live = robot_pose_2_marker(pose_3D_transformed, (1.0, float(i)/len(multi_points_array), 0.0, 1.0))   # Red to yellow
          self.target_rect_array.append(marker_live)
          marker_id += 2

      if self.rect_runmean_i < (self.runmean_lenght-1):
        self.rect_runmean_i += 1
      else:
        self.rect_runmean_i = 0

    self.overlay_msg.text += "--------------------------------------\n"

    # If there are new points values
    if single_point_array.any():
      # Remove overflow points
      if len(single_point_array) > self.point_max:
        single_point_array = np.delete(single_point_array, np.s_[self.point_max:], 0)

      for i, point in enumerate(single_point_array):
        self.point_runmean_array[self.point_runmean_i,i,:] = point
        self.point_runmean_mean[i,:] = np.mean(self.point_runmean_array[:,i,:], 0)
        self.point_runmean_var[i,:] = np.var(self.point_runmean_array[:,i,:], 0)
        self.point_runmean_std[i] = round(np.sqrt(np.sum(self.point_runmean_var[i,:])/(self.point_runmean_var[i,:].size)), 2)   #https://stats.stackexchange.com/questions/25848/how-to-sum-a-standard-deviation

        self.overlay_msg.text += "Point #{0} (mm) [std:{1}]:\n" .format(i, self.point_runmean_std[i])
        
        depth = depth_img_use[(point[1]-1), (point[0]-1)]
        # self.overlay_msg.text += "- Depth:{0}" .format(depth)
        point_3D = [round(num*depth) for num in self.camera_model.projectPixelTo3dRay(point)]
        self.overlay_msg.text += "{1}\n" .format(i, point_3D)
        pose_3D = create_cam_frame_pose(point_3D, "map", marker_id)
        pose_3D_transformed = do_robot_transform(pose_3D, "base_link", "camera_R435i_link")
        self.target_marker_array.append(robot_pose_2_marker(pose_3D_transformed, (0.0, float(i)/len(single_point_array), 1.0, 1.0)))   # Blue 
        marker_id += 2

      if self.point_runmean_i < (self.runmean_lenght-1):
        self.point_runmean_i += 1
      else:
        self.point_runmean_i = 0

    try:
      self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_gray, "mono8"))
      self.image_pub_clahe.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_clahe, "mono8"))
      self.image_pub_rosin.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_rosin, "mono8"))
      self.image_pub_2ponce.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_2ponce, "mono8"))
      self.image_pub_rectified.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_rectified, "rgb8"))

      # Update interactive markers positions
      if len(self.target_rect_array) > 0 or len(self.target_marker_array) > 0:
        interactive_makers = marker_array_2_interactive_marker([self.target_rect_array, self.target_marker_array], "base_link", ["rect", "point"], "targets_points_to_sand")
        self.target_interact_server.insert(interactive_makers, self.interactive_marker_feedback)
        self.target_interact_server.applyChanges()

        self.target_rect_array = []   #Reset list
        self.target_marker_array = []   #Reset list

      # Publish -> "simple" marker array
      # if len(self.target_rect_array) > 0:
      #   self.target_rect_pub.publish(self.target_rect_array)
      #   self.target_rect_array = []   #Reset list

      # if len(self.target_marker_array) > 0:
      #   self.target_point_pub.publish(self.target_marker_array)
      #   self.target_marker_array = []   #Reset list

      self.points_info.publish(self.overlay_msg)
    except CvBridgeError as e:
      print(e)
    

def main(args):
  rospy.init_node('image_converter')
  ic = image_converter()

  rospy.spin()

  # r = rospy.Rate(10) # 10hz
  # while not rospy.is_shutdown():
  #   r.sleep()
   
  # cv2.destroyAllWindows()  # Not realy needed -> safety for cv2.imgshow()


if __name__ == '__main__':
    main(sys.argv)