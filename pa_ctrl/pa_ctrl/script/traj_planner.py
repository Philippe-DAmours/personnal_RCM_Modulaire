#!/usr/bin/env python3

from datetime import datetime
import ransac_plane as pyrsc
import logging

import rospy
import rosbag
import cv2
import socket

import ros_numpy
import math
import tf2_ros
from tf2_ros import transform_listener,buffer, tf2_ros
import message_filters
import tf2_geometry_msgs
import moveit_msgs.msg
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from std_msgs.msg import Empty, Int16, String

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, JointState
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, PoseArray,Point
from moveit_msgs.srv import GetPositionIK,GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState, JointConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from opencv_apps.msg import RotatedRectStamped, RotatedRect, Contour 
import math

### For force schedule selection
#HOST = "192.168.1.100" ###DONE: Make it a variable with robot_ip ros param self.host
PORT = 11006 ###TODO:This shouldn't be here

class omni_rect:
    def __init__(self) -> None:
        self.angle = float()
        self.x = float()
        self.y = float()
        self.lenght = float()
        
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

class traj_planner:
    ### 
    def __init__(self):

        ### Initiation the node -----------------------------------------------------------------
        rospy.init_node("traj_planer_node")
        rospy.loginfo("traj planner initializing")

        ### Callback ----------------------------------------------------------------------------
        rospy.Subscriber("/bounding_box", RotatedRect,self.callbackBOX,queue_size=10)
        rospy.Subscriber("/joint_states",JointState,self.currentJointCallback,queue_size=10) 
        ##rospy.Subscriber("/camera/depth/color/points",PointCloud2, self.callbackPC) ### ORIGINAL DE SAM POUR rs_camera.launch
        rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.callbackPC) ### Pour rgbd_launch
        rospy.Subscriber("/point_cloud_plane",Marker,self.callbackMARKER)
        rospy.Subscriber("/force_schedule",Int16,self.callbackForceSchedule) ### For sending integer to robot for force schedule selection

        ### Callback for the console command made by the operator ------------------------------
        rospy.Subscriber("/console_traj_planner", Empty, self.consoleRunAlgoAndPub,queue_size=10)
        rospy.Subscriber("/console_go_to_marker",Empty,self.callbackGoToMarker)
        rospy.Subscriber("/console_pose_marker",Empty,self.consolePoseArrayToMarker)

        ### Service Proxy ----------------------------------------------------------------------        
        self.compute_ik_client = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        ### Topic Publisher --------------------------------------------------------------------
        self.traj_pub = rospy.Publisher("/joint_path_command",JointTrajectory, queue_size=1)
        self.pose_marker_pub = rospy.Publisher("/pose_marker_array",MarkerArray,queue_size=10)
        self.corrected_plane_marker_pub = rospy.Publisher("/corrected_plane_marker",Marker,queue_size=1)
        self.ransac_debug_pub = rospy.Publisher("/ransac_debug",String,queue_size=1)
        
        #### Class Variable --------------------------------------------------------------------
        self.rect               = RotatedRect()
        self.new_rect           = RotatedRect()
        self.pose_array_        = PoseArray()
        self.point_             = Point()
        self.right_rect_        = RotatedRect()
        self.left_rect_         = RotatedRect()
        self.center_rect_       = RotatedRect()
        self.rect               = RotatedRect()
        self.points             = []
        self.current_joint_pose = JointState()
        self.full_trajectory    = JointTrajectory()
        self.force_schedule     = 10
        self.last_joint_solution= 0
        self.fraction_effective_whith = 0.85

        self.pose_array_.header.frame_id = "camera_R435i_link"


        ### ROS param ---------------------------------------------------------------------------
        if rospy.has_param("/traj_planner/robot_ip"):
            self.host = rospy.get_param("/traj_planner/robot_ip")
        else:
            self.host = "192.168.1.100"

        if rospy.has_param("/camera/realsense2_camera/depth_width"):
            self.img_dim_x = rospy.get_param("/camera/realsense2_camera/depth_width")
        else:
            self.img_dim_x = 1280
        if rospy.has_param("/camera/realsense2_camera/depth_height"):
            self.img_dim_y = rospy.get_param("/camera/realsense2_camera/depth_height")
        else:
            self.img_dim_y = 720

        self.pc_raw = np.zeros((self.img_dim_y,self.img_dim_x, 3), dtype=float)
        
        

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


    def callbackPC(self, data):
        ### Update the current buffer point cloud 
        self.pc_raw, self.pc_dtype, self.pc_header_raw = self.rospc_to_nppc(data)
        rospy.loginfo_once("Traj Planer : callbackPC was here")

    def callbackBOX(self, data):
        self.new_rect = data
        rospy.loginfo_once("Traj Planer : callbackBox was here")
        ### WARNING : opencv_apps.msg RotatedRect() is not the same as cv2.minrect

    def currentJointCallback(self,data):
        rospy.loginfo_once("Traj Planer : JointCallback was here")
        self.current_joint_pose = data
        rospy.loginfo_once(self.current_joint_pose)


    def callbackForceSchedule(self,data):
        rate = rospy.Rate(100)
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
                s.connect((self.host, PORT))
                s.sendall(data.data.to_bytes(16,byteorder='big'))

                rospy.sleep(0.1)
                s.close()

        except ConnectionRefusedError:
            print("Connection to", self.host, "on port", PORT, "refused.")
        
    def consoleRunAlgoAndPub(self,data):
        ### Publish force schedule integer ------------------------------------------------------
        #self.publishschedule()
        ### -------------------------------------------------------------------------------------

        ### Compute the array PoseArray() from rectangle and Point cloud ------------------------
        ### Check rectangle
        if not self.new_rect.size.width or not self.new_rect.size.height:
            rospy.logwarn("No rect has been recorded yet")
            return
        ### Check point cloud
        if np.all(self.pc_raw == 0):
            rospy.logwarn("Empty point cloud. Cancel")
            return
        else:
            self.RectangleSegmentation()
        ### -------------------------------------------------------------------------------------

        ### Marker for Rviz visualisation
        self.PoseArrayToArrowMarker()


        ### Call IK to transform cartesian positions in camera frame to joint command -----------
        ### Check if Position array is filled
        if not self.pose_array_.poses:
            rospy.logwarn("Empty Poses. Cancel IKSrvCall")
            return

        ### From PoseArray to self.full_trajectory
        self.PoseArrayToIKSrvCall()
        ### -------------------------------------------------------------------------------------

        ### Publish self.fulltraject to ROS industrial node -------------------------------------
        self.PublishTraj()
        ### -------------------------------------------------------------------------------------
        

    def publishschedule(self):
        ###TODO: Send schedule int bypassing console callback
        #self.callbackForceSchedule(self.force_schedule)
        pass

    def fit_plane(self,point_cloud): ### Only an exemple, copy-pastedd to ransac
        ### Exemple from 
        ### https://programming-surgeon.com/en/fit-plane-python
        ### Also known as principle components analysis(PCA)
        """
        input
            point_cloud : list of xyz values numpy.array
        output
            plane_v : (normal vector of the best fit plane)
            com : center of mass
        """
        point_cloud = point_cloud.reshape(-1, point_cloud.shape[-1])

        cleaned_points = point_cloud[np.all(np.isfinite(point_cloud), axis=1)]

        com = np.sum(cleaned_points, axis=0,dtype=np.float64) / len(cleaned_points)
        # calculate the center of mass
        q = cleaned_points - com
        # move the com to the origin and translate all the points (use numpy broadcasting)
        Q = np.dot(q.T, q)
        # calculate 3x3 matrix. The inner product returns total sum of 3x3 matrix
        la, vectors = np.linalg.eig(Q)
        # Calculate eigenvalues and eigenvectors
        plane_v = vectors.T[np.argmin(la)]
        # Extract the eigenvector of the minimum 

        #if plane_v[2] <= 0 :
        #   print("\033[93mWARNING:Normal is negative\033[0m")
        #    plane_v = np.negative(plane_v)

        return plane_v, com

        
    def EstimateXYresolution(self):
        pc_full = self.pc_raw
        pc = pc_full[..., :3]

        ### resolution mm/px . Make sure every pixel is on the wall.
        self.camera_resolutionX = round(1000 * abs(pc[0,-1,0] - pc[0,0,0]) / pc.shape[1], 2)
        self.camera_resolutionY = round(1000 * abs(pc[-1,0,1] - pc[0,0,1]) / pc.shape[0], 2)
        ### if resolution X and Y, the wall perfectly perpendicular to camera


        if self.camera_resolutionX < 0.1 or self.camera_resolutionY < 0.1:
            rospy.logwarn('Warning impossible resolution:')
            rospy.logwarn("x: {0}, y: {1}\n" .format(self.camera_resolutionX , self.camera_resolutionY)) 
            rospy.logwarn("arbitrary 1.0 - 1.0 mm is used instead")
            self.camera_resolutionX = 1
            self.camera_resolutionY = 1

        self.average_camera_resolution = (self.camera_resolutionX+self.camera_resolutionY)/2
        
    def callbackMARKER(self, data):
        self.marker = data
        rospy.loginfo_once("Traj Planer : CallbackMARKER was here")
        self.marker.header.frame_id = "camera_R435i_link"
        self.corrected_plane_marker_pub.publish(self.marker)
        

        

    def callbackGoToMarker(self,data):
        rospy.loginfo(self.marker.pose)
        rospy.loginfo_once("Traj Planer : callbackGoToMarker was here")
        ##self.marker.header.frame_id = "camera_R435i_link"
        tfbuffer = tf2_ros.Buffer(rospy.Duration(100.0))
        
        
        listener = tf2_ros.TransformListener(tfbuffer)
        
        tf_pose = tf2_geometry_msgs.PoseStamped()
        tf_pose.pose = self.marker.pose
        tf_pose.pose.position.z -= 0.01

        transform = tfbuffer.lookup_transform("world",
                                   "camera_R435i_link", #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0))
        self.pose_transformed = tf2_geometry_msgs.do_transform_pose(tf_pose, transform)
        rospy.loginfo(self.pose_transformed)
        ##self.computeIKSrvCall(self.pose_transformed.pose)
        self.computeIKSrvCall(self.marker.pose)
        self.PublishTraj()
        



    def GoToMarker(self):
        ### Publish joint trajectory to /joint_path_command
        self.computeIKSrvCall(self.marker.pose)

        self.PublishTraj()
        
        

    def RectangleSegmentation(self):
        
        ### Freeze rectangle
        #self.rect = RotatedRect() ### DEV: Uncomment for auto complete
        self.rect = self.new_rect ### Comment for dev auto complete

        

        rospy.loginfo(self.new_rect)
        
        ### Estimate resolution
        self.EstimateXYresolution()

        ### The diameter of the tool
        ### The diameter is elipsed if the wall is not perpendicular enough
        

        if not(abs(self.camera_resolutionX - self.camera_resolutionY) <= 2) :
            ### The wall is not perpendicular enough
            rospy.logwarn("The wall is not perpendicular enough")
            rospy.logwarn(self.camera_resolutionX)
            rospy.logwarn(self.camera_resolutionY)
            return
            
            
        

        self.D_sander_mm = 225 # (mm)
        D_sander_px_x = self.D_sander_mm / self.camera_resolutionX ### (mm)/(mm/px)
        D_sander_px_y = self.D_sander_mm / self.camera_resolutionY
        self.Effective_width_px = (D_sander_px_x + D_sander_px_y)/2 * self.fraction_effective_whith 
        self.Effective_width_mm = self.D_sander_mm * self.fraction_effective_whith


        ### Le premier point de la boite est le point le plus bas (+y), puis clockwise
        ### Faire attention quand le point tombe est egale au 2e point
        ### L'angle [-90,0) du plus bas point [0,] vers point 3
        
            
        ### We can asume mostly all rectangle are -90 or 0
        ### check if joint is vertical with width and heigh


        ### TODO: La condition if suivant pourrait être fait un seul case 
        if (self.rect.angle>-5 and self.rect.size.width<=self.rect.size.height):
            ### joint vertical
            rospy.loginfo("Verticle joint")
            self.rect_type = 0
            self.rect_type_angle = 90
            
            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width_px/2)
            self.right_rect_.center.y = self.rect.center.y + math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width_px/2)
            self.right_rect_.size.width  = self.Effective_width_px
            self.right_rect_.size.height = self.rect.size.height
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width_px/2)
            self.left_rect_.center.y = self.rect.center.y - math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width_px/2)
            self.left_rect_.size.width  = self.Effective_width_px
            self.left_rect_.size.height = self.rect.size.height
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.Effective_width_px
            self.center_rect_.size.height = self.rect.size.height

            
        elif (self.rect.angle<-85 and self.rect.size.width>=self.rect.size.height):
            ### joint vertical but fliped up right
            rospy.loginfo("Verticle joint")
            self.rect_type = 1
            self.rect_type_angle = 0

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x - math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.center.y = self.rect.center.y + math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.size.width  = self.rect.size.width
            self.right_rect_.size.height = self.Effective_width_px
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x + math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.center.y = self.rect.center.y - math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.size.width  = self.rect.size.width
            self.left_rect_.size.height = self.Effective_width_px
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.rect.size.width
            self.center_rect_.size.height = self.Effective_width_px
            

        elif (self.rect.angle>-5 and self.rect.size.width>=self.rect.size.height) :
            ### joint horizontal 
            rospy.loginfo("Horizontal joint")
            self.rect_type = 2
            self.rect_type_angle = 180

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.center.y = self.rect.center.y + math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.size.width  = self.rect.size.width
            self.right_rect_.size.height = self.Effective_width_px
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.center.y = self.rect.center.y - math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.size.width  = self.rect.size.width
            self.left_rect_.size.height = self.Effective_width_px
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.rect.size.width
            self.center_rect_.size.height = self.Effective_width_px

            
        elif (self.rect.angle<-85 and self.rect.size.width<=self.rect.size.height):
            ### joint horizontal
            rospy.loginfo("Horizontal joint")
            self.rect_type = 3
            self.rect_type_angle = 270

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.center.y = self.rect.center.y + math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.right_rect_.size.width  = self.Effective_width_px  
            self.right_rect_.size.height = self.rect.size.width
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.center.y = self.rect.center.y - math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width_px/2)
            self.left_rect_.size.width  = self.Effective_width_px      
            self.left_rect_.size.height = self.rect.size.height
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width  = self.Effective_width_px      
            self.center_rect_.size.height = self.rect.size.height


        else:
            ### Erreur : Joint en diagonal
            rospy.logwarn("Erreur: Unexpected rect orrientation")
            return

        self.rects = []

        omni_rect_ = omni_rect()
        omni_rect_.angle = self.left_rect_.angle -self.rect_type_angle
        omni_rect_.lenght = max(self.left_rect_.size.height,self.left_rect_.size.width)
        omni_rect_.x = self.left_rect_.center.x
        omni_rect_.y = self.left_rect_.center.y
        self.rects.append(omni_rect_)

        omni_rect_ = omni_rect()
        omni_rect_.angle = self.left_rect_.angle -self.rect_type_angle -180
        omni_rect_.lenght = max(self.left_rect_.size.height,self.left_rect_.size.width)
        omni_rect_.x = self.right_rect_.center.x
        omni_rect_.y = self.right_rect_.center.y
        self.rects.append(omni_rect_)

        

        ##rospy.loginfo("rect type : " + str(self.rect_type))
        ##rospy.loginfo("right_rect \n" + "angle:"+str(self.right_rect_.angle) + "\n x:" + str(self.right_rect_.center.x) + "\n y:" + str(self.right_rect_.center.y))
        ##rospy.loginfo("left_rect \n" + "angle:"+str(self.left_rect_.angle) + "\n x:" + str(self.left_rect_.center.x) + "\n y:" + str(self.left_rect_.center.y))
        ##rospy.loginfo("angle:"+str(self.center_rect_.angle) + "\n x:" + str(self.center_rect_.center.x) + "\n y:" + str(self.center_rect_.center.y))

        ### Check the size of the rectangle to segmente in 3 or 2
        ### TODO: Add case for 1 and 4 pass for better blend in
        ### TODO: Change from pixel to mm
        ## if (self.rect.size.width > self.Effective_width_px * 3 ) and (self.rect.size.height > self.Effective_width_px * 3 ):
        if (self.rect.size.width * self.average_camera_resolution > self.Effective_width_mm * 3 ) and (self.rect.size.height*self.average_camera_resolution > self.Effective_width_mm * 3 ):

            ### The rectangle is too wide for 3 pass.
            rospy.logwarn("Unexpectly large joint")
            rospy.logwarn("Too large joint. Use center pass for the moment")

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.rects[-1].angle +180
            omni_rect_.lenght = max(self.left_rect_.size.height,self.left_rect_.size.width)
            omni_rect_.x = self.center_rect_.center.x
            omni_rect_.y = self.center_rect_.center.y
            self.rects.append(omni_rect_)
            
        ## elif (self.rect.size.width > self.Effective_width_px * 2 ) and (self.rect.size.height > self.Effective_width_px * 2 ):
        elif (self.rect.size.width * self.average_camera_resolution > self.Effective_width_mm * 2 ) and (self.rect.size.height * self.average_camera_resolution > self.Effective_width_mm * 2 ):
        
            ### The rectangle is big enough to subdivided in 3 part
            rospy.loginfo("Large joint. Use the center pass")

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.rects[-1].angle +180
            omni_rect_.lenght = max(self.left_rect_.size.height,self.left_rect_.size.width)
            omni_rect_.x = self.center_rect_.center.x
            omni_rect_.y = self.center_rect_.center.y
            self.rects.append(omni_rect_)
            
            
        ## elif (self.rect.size.width > self.Effective_width_px * 1 ) and (self.rect.size.height > self.Effective_width_px * 1 ):
        elif (self.rect.size.width * self.average_camera_resolution > self.Effective_width_mm * 1 ) and (self.rect.size.height * self.average_camera_resolution > self.Effective_width_mm * 1 ):

            ### The rectangle is small enough for 2 pass
            rospy.loginfo("The rectangle is small enough. Use 2 pass")
            

        ##elif (self.rect.size.width < self.Effective_width_px) and (self.rect.size.height < self.Effective_width_px):
        elif (self.rect.size.width *self.average_camera_resolution < self.Effective_width_mm) and (self.rect.size.height * self.average_camera_resolution < self.Effective_width_mm):

            ### The rectangle is too small for one pass
            rospy.logwarn("The rectangle is too small")
        else :
            ### Something went wrong
            rospy.logwarn("Something went wrong with rectangle size")

         ### Separer le point cloud pour les points dans le rectangle
         # Example : pc[1:4][1:4]
         #box = cv2.boxPoints(self.right_rect_)
         

        self.points = []
        
        rospy.loginfo("rectangles array")

        for rect in self.rects :
            ##rospy.loginfo("angle:"+str(rect.angle) + "\n x:" + str(rect.x) + "\n y:" + str(rect.y))
            
            point = Point()
            point.x = rect.x - math.cos(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px)/2
            point.y = rect.y - math.sin(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px)/2 
            self.points.append(point)
            point = Point()
            point.x = rect.x + math.cos(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px)/2
            point.y = rect.y + math.sin(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px)/2
            self.points.append(point)
            point = Point()
            point.x = rect.x + math.cos(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px+200)/2
            point.y = rect.y + math.sin(math.radians(rect.angle))*(rect.lenght-self.Effective_width_px+200)/2
            self.points.append(point)
            
        ### Let's do orientation by localy checking the point cloud
        ### Make sure no point is out of the plaster by taking a small square

        self.pose_array_ = PoseArray()
        self.pose_array_.header.frame_id = "camera_R435i_link"

        for point in self.points:
            self.PointToPose(point)


    def PointToPose(self,point):
        ### Take the point, tranform it to pose and append it to a PoseArray()
        pc_full = self.pc_raw
        pc = pc_full[..., :3]

        side = self.Effective_width_px*math.sqrt(2)/2
        side = np.int0(side)
        point.x = np.int0(point.x)
        point.y = np.int0(point.y)

        ### Check if the point is out of bound
        if (point.x >= self.img_dim_x):
            rospy.logwarn("Point is out of x bound :" + str(point.x))
            point.x = self.img_dim_x-1
        elif(point.x < 0):
            rospy.logwarn("Point is out of x bound :" + str(point.x))
            point.x = 0

        if (point.y >= self.img_dim_y):
            rospy.logwarn("Point is out of y bound :" + str(point.y))
            point.y = self.img_dim_y-1
        elif(point.y < 0):
            rospy.logwarn("Point is out of y bound :" + str(point.y))
            point.y = 0

        ##rospy.loginfo("Point of interest")
        ##rospy.loginfo(point)
        ##rospy.loginfo(side)
        ##sub_pc = np.zeros((2*side,2*side,3),dtype=float)
        sub_pc = pc[point.y - side:point.y + side,point.x -side:point.x + side,:]

        ### Orientation par RANSAC -----------------------------------------------------
        plane = pyrsc.Plane()
        ### Keep minPoint as high as possible and threashold over 0.0035
        ### if RANSAC take too much time, augment threshold and reduce max iteration
        ### if bad orientation, reduce threshold and augment max iteration

        if np.size(sub_pc) == 0:
            rospy.loginfo("RANSAC done with all point")
            best_eq, best_inliers, perc_success, it_success = plane.fit(pc, 0.0037, minPoints=int(pc.shape[0]*pc.shape[1]*0.99), maxIteration=100)#100
            ref_normal = self.fit_plane(pc)
        elif np.size(sub_pc)!=0:
            rospy.loginfo("RANSAC done with points near pose")
            best_eq, best_inliers, perc_success, it_success = plane.fit(sub_pc, 0.0035, minPoints=int(sub_pc.shape[0]*sub_pc.shape[1]*0.99), maxIteration=300)#300  # Good compromise
            ref_normal = self.fit_plane(pc)
        else:
            ### Unexpected outcom
            rospy.logwarn("Unexpected condition")
            return
        ##best_eq, best_inliers, perc_success, it_success = plane.fit(pc, 0.004, minPoints=int(pc.shape[0]*pc.shape[1]*0.9), maxIteration=100)
        best_eq = np.array(best_eq)

        rospy.loginfo("RANSAC done with " + str(it_success) + " iteration.")
        rospy.loginfo("RANSAC done with " + str(perc_success) + " succes rate.") ### should be move than 95, up threashold and max iteration
        
        if best_eq.size == 0:  # no planes found
          rospy.logwarn("WARNING: RANSAC found no plane")
          best_eq = np.array([1,1,1,1])  # https://pypi.org/project/pyransac3d/
        elif(best_eq[3] > 0):
          best_eq = -best_eq
        ### -----------------------------------------------------------------

        ### Alternative code faster than RANSAC ------------------------------
        ### Orientation by principle component analysis (PCA)
        ### Warning : PCA doesn't ingnore aberent data
        ### Exemple from https://programming-surgeon.com/en/fit-plane-python/
        # pts_listALL = pc.reshape(-1, pc.shape[-1]) 
        # cleaned_points = pts_listALL[np.all(np.isfinite(pts_listALL), axis=1)]
        # com = np.sum(cleaned_points, axis=0) / cleaned_points.shape[0]
        # # calculate the center of mass
        # q = cleaned_points - com
        # # move the com to the origin and translate all the points (use numpy broadcasting)
        # Q = np.dot(q.T, q)
        # # calculate 3x3 matrix. The inner product returns total sum of 3x3 matrix
        # la, vectors = np.linalg.eig(Q)
        # # Calculate eigenvalues and eigenvectors
        # plane_v = vectors.T[np.argmin(la)]
        # # Extract the eigenvector of the minimum 
        ### ------------------------------------------------------------------

        ### send to topic for debug -----------------------------------------
        #self.ransac_debug_pub.publish("fit_plane reference : {ref_normal}")
        #self.ransac_debug_pub.publish("ransac plane fit    : {best_eq}")
        #self.ransac_debug_pub.publish("RANSAC done with " + str(it_success) + " iteration.")
        #self.ransac_debug_pub.publish("RANSAC done with " + str(perc_success) + " succes rate.")
        self.ransac_debug_pub.publish("fit_plane reference : "+str(ref_normal) + "\nransac plane fit    : "+str(best_eq)+"\nRANSAC done with " + str(it_success) + " iteration\n"+"RANSAC done with " + str(perc_success) + " succes rate.")
        ### -----------------------------------------------------------------

        ### Plane equation to orientaiton -----------------------------------
        n = best_eq[:3]
        n_norm = np.linalg.norm(n)
        n_normalized = n / n_norm
        unitZ = np.array([0,0,1])
        distance = best_eq[3] / n_norm
        center_point = -1 * distance * n_normalized  # Not centered with point cloud... DONE: better fix than middle pixel of the image

        orientation = np.cross(unitZ, n_normalized)
        orientation = np.append(orientation, np.dot(unitZ, n_normalized) + np.sqrt(np.linalg.norm(unitZ)**2 + n_norm**2))
        orientation_normalized = orientation / np.linalg.norm(orientation)
        ### ------------------------------------------------------------------

        


        ##self.EstimateXYresolution() ### Already done in rectangle segmentation

        self.pose = Pose()
        
        self.pose.position.x    = pc[point.y,point.x,0] ### The orders of x and y need to be changed
        self.pose.position.y    = pc[point.y,point.x,1]
        self.pose.position.z    = pc[point.y,point.x,2] -0.025 ### Retract 10mm from the wall in camera frame
        self.pose.orientation.x = orientation_normalized[0]
        self.pose.orientation.y = orientation_normalized[1]
        self.pose.orientation.z = orientation_normalized[2]
        self.pose.orientation.w = orientation_normalized[3]

        ### Override orientation with marker orientation(Orientation of the full RANSAC)
        ##self.pose.orientation = self.marker.pose.orientation

        
        self.pose_array_.poses.append(self.pose)

    def PoseArrayToMarker(self):

        if not self.pose_array_.poses:
            rospy.logwarn("PoseArray empty. Do rectangle segmentation first")
            return
        
        marker_array = MarkerArray()
        
        for idx,pose in enumerate(self.pose_array_.poses):
            marker = Marker()
            marker.id = idx
            marker.pose = pose
            marker.header.frame_id = "camera_R435i_link"
            marker.type = 3 ### Cylinder
            ##marker.type = marker.TEXT_VIEW_FACING
            
            marker.scale.x = self.D_sander_mm/1000*0.85
            marker.scale.y = self.D_sander_mm/1000*0.85
            marker.scale.z = 0.01
            marker.color.r = 1.0 - 0.1*idx
            marker.color.g = 0.0
            marker.color.b = 0.0 + 0.1*idx
            marker.color.a = 1.0 
            marker.frame_locked = False
            ##marker.text = str(idx)
            marker_array.markers.append(marker)
        
        self.pose_marker_pub.publish(marker_array)
            
    def PoseArrayToArrowMarker(self):

        if not self.pose_array_.poses:
            rospy.logwarn("PoseArray empty. Do rectangle segmentation first")
            return
        
        marker_array = MarkerArray()
        
        
        for idx in range(len(self.pose_array_.poses)-1):
            marker = Marker()
            marker.id = idx
            # marker.pose = pose
            start_pose = self.pose_array_.poses[idx]
            end_pose = self.pose_array_.poses[idx+1]
            marker.points = [start_pose.position,end_pose.position]
            marker.header.frame_id = "camera_R435i_link"
            marker.type = 0 ### Arrow
            ##marker.type = marker.TEXT_VIEW_FACING
            
            #marker.scale.x = self.D_sander_mm/1000*0.85
            #marker.scale.y = self.D_sander_mm/1000*0.85
            #marker.scale.z = 0.01
            marker.scale.x = 0.02
            marker.scale.y = 0.04
            marker.color.r = 1.0 - 0.1*idx
            marker.color.g = 0.0
            marker.color.b = 0.0 + 0.1*idx
            marker.color.a = 1.0 
            marker.frame_locked = False
            ##marker.text = str(idx)
            marker_array.markers.append(marker)
        
        self.pose_marker_pub.publish(marker_array)

    def consolePoseArrayToMarker(self,data):
        ### Callback for the user to test PoseArraytoMarker
        if not self.pose_array_.poses:
            self.RectangleSegmentation()

        self.PoseArrayToMarker()
    

    def PublishTraj(self):
        ### Publish the full trajectory msg to joint_path_command
        self.traj_pub.publish(self.full_trajectory)
        rospy.loginfo("published traj")
        rospy.loginfo(self.full_trajectory)

        ### Clear trajectory_msg and last solution
        self.full_trajectory = JointTrajectory()
        self.last_joint_solution = 0

    def PoseArrayToIKSrvCall(self):
        ### Take the PoseArray and call IK service for each PoseStamped
        
        ### Reset the trajectory
        self.full_trajectory = JointTrajectory()

        for pose in self.pose_array_.poses:
            self.computeIKSrvCall(pose)


    def computeIKSrvCall(self,pose):
        ### PoseStamped() to joint_pose
        req = PositionIKRequest() ### Take only one PoseStamped() per request
        req.group_name = "manipulator"
        
        ### check if there was a last joint solution
        if self.last_joint_solution:
            rospy.loginfo("There was a last joint solution")
            req.robot_state.joint_state = self.last_joint_solution
        else:
            req.robot_state.joint_state = self.current_joint_pose
        

        req.ik_link_name = "TCP"
        req.pose_stamped.pose = pose
        req.pose_stamped.header.frame_id = "camera_R435i_link"
        ##rospy.loginfo("Trying IK for pose")
        ##rospy.loginfo(req.pose_stamped)

        ### Override orrientation for debuging
        ##req.pose_stamped.pose.orientation.x = 0.70700
        ##req.pose_stamped.pose.orientation.y = -0.023
        ##req.pose_stamped.pose.orientation.z = 0.70711
        ##req.pose_stamped.pose.orientation.w = -0.0698

        req.avoid_collisions = True
        
        #req.timeout = rospy.Duration(10)
        

        ### Ajoutons une joint constraint pour que l'axe du husky soit à zéro puisque la démo se fais sans cette axe.
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "husky_to_robot_base_plate"
        ##joint_constraint.position = 0.01 ### Note : Donner au IK solver en metre
        joint_constraint.position = 0.550 ### Note : Donner au IK solver en metre
        ## TODO: Joint_constraint give unstable result, verifie where current joint get changed
        ##joint_constraint.position = self.current_joint_pose.position[6]
        joint_constraint.tolerance_below = 0.005
        joint_constraint.tolerance_above = 0.005
        joint_constraint.weight   = 100
        req.constraints.joint_constraints.append(joint_constraint)
        

        ### Ajoutons des contraintes autour des joints pour 
        ### évité des flip flop de solution
        pi = 3.1416
        
        #joint_constraint.joint_name = "joint_1"
        #joint_constraint.position = 0.0
        #joint_constraint.tolerance_below = pi/2
        #joint_constraint.tolerance_above = pi/2
        #req.constraints.joint_constraints.append(joint_constraint)

        #joint_constraint.joint_name = "joint_2"
        #joint_constraint.position = pi/4
        #joint_constraint.tolerance_below = pi/4
        #joint_constraint.tolerance_above = pi/4
        #req.constraints.joint_constraints.append(joint_constraint)

        joint_constraint3 = JointConstraint()
        joint_constraint3.weight = 1
        joint_constraint3.joint_name = "joint_3"
        joint_constraint3.position = 0.0
        joint_constraint3.tolerance_below = pi/2
        joint_constraint3.tolerance_above = pi/2
        req.constraints.joint_constraints.append(joint_constraint3)
        
        ### Configuration du joint 4 à gauche du 
        joint_constraint4 = JointConstraint()
        joint_constraint4.weight = 1
        joint_constraint4.joint_name = "joint_4"
        joint_constraint4.position = 0.0
        joint_constraint4.tolerance_below = pi/3
        joint_constraint4.tolerance_above = pi/3
        req.constraints.joint_constraints.append(joint_constraint4)

        #joint_constraint.joint_name = "joint_5"
        #joint_constraint.position = 0.0
        #joint_constraint.tolerance_below = pi/2
        #joint_constraint.tolerance_above = pi/2
        #req.constraints.joint_constraints.append(joint_constraint)

        ### Clear joint_constraintes for debuging
        ##req.constraints.joint_constraints.clear()


        ##rospy.loginfo(req)
        ### call le service
        res = self.compute_ik_client(req)
        ##rospy.loginfo(res.error_code.val)

        
        while not res.error_code.val == 1:
            rospy.loginfo(res.error_code)
            rospy.logwarn("Did not find solution for IK. Looping back")
            rospy.logwarn("The position request was :")
            rospy.loginfo(req.pose_stamped)
            rospy.loginfo("Constrain :\n"+str(self.current_joint_pose.position[6]))
            res = self.compute_ik_client(req)
            
        green_start = '\033[92m'
        color_reset = '\033[0m'
        rospy.loginfo(green_start + "Found IK solution in " + color_reset)

        ### Reponse du service call
        self.joint_pose = res
    
        
        ### Put joint_pose in a trajectory msg
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration(0.3)
        #trajectory_point.positions = res.solution.joint_state.position
    
        ### Tuple object does not support assigment --------------------------------------
        my_list = (res.solution.joint_state.position)
        #my_list = (my_list[0]*1000.0,)+ my_list[1:]
        #my_list[0] = 1.0        
        trajectory_point.positions = list(my_list)

        # my_joint = res.solution.joint_state.position[0]
        # my_joint = my_joint * 1000
        # rospy.loginfo("my_joint " + str(my_joint))
        rospy.loginfo("IK 7e axe " + str(res.solution.joint_state.position[0]))
        # trajectory_point.positions[0] = my_joint

        rospy.loginfo_once(trajectory_point.positions)
        
        #my_list = trajectory_point.positions
        #my_list = list(my_list)
        #my_list[0] = my_joint
        #trajectory_point.positions = tuple(my_list)
        #rospy.loginfo_once(trajectory_point.positions)

        self.full_trajectory.header = res.solution.joint_state.header
        ### WARNING: Make sure joint name are in the correct order depending if the state are from
        self.full_trajectory.joint_names = self.joint_pose.solution.joint_state.name
        ##self.full_trajectory.joint_names = self.current_joint_pose.name
        ##rospy.logwarn(self.full_trajectory.joint_names)

        ### NOTE: FANUC transforme l'array de rad2deg
        ### Mettons le premiere joint prismatic en mm°/rad
        trajectory_point.positions[0] = 1000.0*trajectory_point.positions[0]
        trajectory_point.positions[0] = np.radians(trajectory_point.positions[0])

        self.full_trajectory.points.append(trajectory_point)

        ### update curent joint with last IK solution
        ##self.current_joint_pose.position = self.joint_pose.solution.joint_state.position
        self.last_joint_solution = JointState()
        self.last_joint_solution = self.current_joint_pose
        self.last_joint_solution.position = self.joint_pose.solution.joint_state.position

        

def main():
    traj_planner_node = traj_planner()
    rospy.spin()

if __name__ == "__main__":
    main()



        












