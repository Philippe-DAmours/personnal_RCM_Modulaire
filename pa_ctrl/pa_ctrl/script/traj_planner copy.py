#!/usr/bin/env python3

import ransac_plane as pyrsc

import rospy
import rosbag
import cv2

import ros_numpy
import math
import tf2_ros as tf2
import message_filters
import tf2_geometry_msgs
import moveit_msgs.msg
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from std_msgs.msg import Empty

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, JointState
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, PoseArray,Point
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, JointConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from opencv_apps.msg import RotatedRectStamped, RotatedRect, Contour 
import math


        


class traj_planner:
    ### 
    def __init__(self):

        rospy.init_node("traj_planer_node")
        rospy.loginfo("traj planner initializing")
        
        rospy.Subscriber("/camera/depth/image_rect_raw",Image, self.callbackPC)

        self.compute_ik_client = rospy.ServiceProxy("/compute_ik", GetPositionIK )
        
        self.traj_pub = rospy.Publisher("/joint_path_command",JointTrajectory, queue_size=1)
        
        self.rect = RotatedRect()
        
        
        self.pose_array_        = PoseArray()
        self.point_             = Point()
        self.right_rect_        = RotatedRect()
        self.left_rect_         = RotatedRect()
        self.center_rect_       = RotatedRect()
        self.rect               = RotatedRect()
        self.points             = []
        self.current_joint_pose = JointState()
        self.full_trajectory    = JointTrajectory()
        
        

    def rospc_to_nppc(self, rospc):
        cloud_tuple = ros_numpy.numpify(rospc)
        rospy.loginfo(cloud_tuple)
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


    def callbackBOX(self, data):
        self.new_rect = data
        ### WARNING : opencv_apps.msg RotatedRect() is not the same as cv2.minrect
        #box = cv2.boxPoints(self.new_rect)
        #box = np.int0(box)
        #self.box = box
        

    def currentJointCallback(self,data):
        self.current_joint_pose = data
        
    def consoleRunAlgoAndPub(self,data):
        ### From self.new_rect to PoseArray()
        self.RectangleSegmentation()
        ### From PoseArray to self.full_trajectory
        self.PoseArrayToIKSrvCall()
        ### Publish self.fulltraject
        self.PublishTraj()



        
    def EstimateXYresolution(self):
        pc_full = self.pc_raw
        pc = pc_full[..., :3]

        self.camera_resolutionX = round(1000 * abs(pc[0,-1,0] - pc[0,0,0]) / pc.shape[1], 2)
        self.camera_resolutionY = round(1000 * abs(pc[-1,0,1] - pc[0,0,1]) / pc.shape[0], 2)
        ### if resolution X and Y, the wall perfectly perpendicular to camera


        if self.camera_resolutionX < 0.1 or self.camera_resolutionY < 0.1:
            rospy.logwarn('Warning impossible resolution:')
            rospy.logwarn("x: {0}, y: {1}\n" .format(self.camera_resolutionX , self.camera_resolutionY)) 
            rospy.logwarn("arbitrary 1.0 - 1.0 mm is used instead")
            self.camera_resolutionX = 1
            self.camera_resolutionY = 1
        
    def callbackMARKER(self, data):
        self.marker = data
        self.point.pose = self.marker.pose

    def callbackGoToMarker(self,data):
        self.GoToMarker()

    def GoToMarker(self):
        ### Publish joint trajectory to /joint_path_command
        self.joint_path_command_pub.publish()
        

    def RectangleSegmentation(self):
        
        ### Freeze rectangle
        self.rect = self.new_rect

        ### Estimate
        self.EstimateXYresolution()

        ### The diameter of the tool
        ### The diameter is elipsed if the wall is not perpendicular
        if not(abs(self.camera_resolutionX - self.camera_resolutionY) <= 2) :
            ### The wall is not perpendicular enough
            rospy.logwarn("The wall is not perpendicular enough")
            return
            
        

        D_sander_mm = 225 # (mm)
        D_sander_px_x = D_sander_mm * self.camera_resolutionX
        D_sander_px_y = D_sander_mm * self.camera_resolutionY
        self.Effective_width = (D_sander_px_x + D_sander_px_y)/2 * 0.85 ### TODO : Determin effective width more regourously


        ### Le premier point de la boite est le point le plus bas (+y), puis clockwise
        ### Faire attention quand le point tombe est egale au 2e point
        ### L'angle [-90,0) du plus bas point [0,] vers point 3
        
            
        ### We can asume mostly all rectangle are -90 or 0
        ### check if joint is vertical with width and heigh

        if (self.rect.angle>-5 and self.rect.size.width<=self.rect.size.height):
            ### joint vertical
            rospy.loginfo("Verticle joint")
            self.rect_type = 0
            
            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.right_rect_.center.y = self.rect.center.y + math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.right_rect_.size.width  = self.Effective_width
            self.right_rect_.size.height = self.rect.size.height
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.left_rect_.center.y = self.rect.center.y - math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.left_rect_.size.width  = self.Effective_width
            self.left_rect_.size.height = self.rect.size.height
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.Effective_width
            self.center_rect_.size.height = self.rect.size.height

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.left_rect_.angle -90
            omni_rect_.lenght = self.left_rect_.size.height
            omni_rect_.x = self.left_rect_.center.x
            omni_rect_.y = self.left_rect_.center.y

            self.rects = [omni_rect_,]

            omni_rect_.angle -= 180
            omni_rect_.x = self.right_rect_.center.x
            omni_rect_.y = self.right_rect_.center.y
            self.rects.append(omni_rect_)

            
        elif (self.rect.angle<-85 and self.rect.size.width>=self.rect.size.height):
            ### joint vertical but fliped up right
            rospy.loginfo("Verticle joint")
            self.rect_type = 1

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x - math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.right_rect_.center.y = self.rect.center.y + math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.right_rect_.size.width  = self.rect.size.width
            self.right_rect_.size.height = self.Effective_width
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x + math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.left_rect_.center.y = self.rect.center.y - math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.left_rect_.size.width  = self.rect.size.width
            self.left_rect_.size.height = self.Effective_width
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.rect.size.width
            self.center_rect_.size.height = self.Effective_width
            

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.left_rect_.angle
            omni_rect_.lenght = self.left_rect_.size.height
            omni_rect_.x = self.left_rect_.center.x
            omni_rect_.y = self.left_rect_.center.y

            self.rects = [omni_rect_,]

            omni_rect_.angle -= 180
            omni_rect_.x = self.right_rect_.center.x
            omni_rect_.y = self.right_rect_.center.y
            self.rects.append(omni_rect_)


        elif (self.rect.angle>-5 and self.rect.size.width>=self.rect.size.height) :
            ### joint horizontal 
            rospy.loginfo("Horizontal joint")
            self.rect_type = 2

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.right_rect_.center.y = self.rect.center.y - math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.right_rect_.size.width  = self.rect.size.width
            self.right_rect_.size.height = self.Effective_width
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.cos(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.left_rect_.center.y = self.rect.center.y + math.sin(math.radians(self.rect.angle))*(self.rect.size.width/2 - self.Effective_width/2)
            self.left_rect_.size.width  = self.rect.size.width
            self.left_rect_.size.height = self.Effective_width
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width = self.rect.size.width
            self.center_rect_.size.height = self.Effective_width

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.left_rect_.angle -270
            omni_rect_.lenght = self.left_rect_.size.height
            omni_rect_.x = self.left_rect_.center.x
            omni_rect_.y = self.left_rect_.center.y

            self.rects = [omni_rect_,]

            omni_rect_.angle -= 180
            omni_rect_.x = self.right_rect_.center.x
            omni_rect_.y = self.right_rect_.center.y
            self.rects.append(omni_rect_)
            
            
        elif (self.rect.angle<-85 and self.rect.size.width<=self.rect.size.height):
            ### joint horizontal
            rospy.loginfo("Horizontal joint")
            self.rect_type = 3

            ### Right rectangle
            self.right_rect_.angle = self.rect.angle
            self.right_rect_.center.x = self.rect.center.x + math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.right_rect_.center.y = self.rect.center.y + math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.right_rect_.size.width  = self.Effective_width  
            self.right_rect_.size.height = self.rect.size.width
            ### Left rectangle
            self.left_rect_.angle = self.rect.angle
            self.left_rect_.center.x = self.rect.center.x - math.cos(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.left_rect_.center.y = self.rect.center.y - math.sin(math.radians(self.rect.angle))*(self.rect.size.height/2 - self.Effective_width/2)
            self.left_rect_.size.width  = self.Effective_width      
            self.left_rect_.size.height = self.rect.size.height
            ### Center rectangle
            self.center_rect_.angle = self.rect.angle
            self.center_rect_.center.x = self.rect.center.x
            self.center_rect_.center.y = self.rect.center.y
            self.center_rect_.size.width  = self.Effective_width      
            self.center_rect_.size.height = self.rect.size.height

            omni_rect_ = omni_rect()
            omni_rect_.angle = self.left_rect_.angle -180
            omni_rect_.lenght = self.left_rect_.size.height
            omni_rect_.x = self.left_rect_.center.x
            omni_rect_.y = self.left_rect_.center.y

            self.rects = [omni_rect_,]

            omni_rect_.angle -= 180
            omni_rect_.x = self.right_rect_.center.x
            omni_rect_.y = self.right_rect_.center.y
            self.rects.append(omni_rect_)


        else:
            ### Erreur : Joint en diagonal
            rospy.logwarn("Erreur: Unexpected joint orrientation")

        ### Check the size of the rectangle to segmente in 3 or 2

        
        
        if (self.rect.size.width > self.Effective_width * 3 ):
            ### The rectangle is too wide for 3 pass.
            self.center_rect_ = RotatedRect() 
            rospy.logwarn("Unexpectly large joint")

            
        elif (self.rect.size.width > self.Effective_width * 2 ):
            ### The rectangle is big enough to subdivided in 3 part
            rospy.loginfo("Use the center rectangle")

            omni_rect_.angle += 180
            omni_rect_.x = self.center_rect_.center.x
            omni_rect_.y = self.center_rect_.center.y
            self.rects.append(omni_rect_)
            
            
        elif (self.rect.size.width > self.Effective_width * 1 ):
            ### The rectangle is small enough for 2 pass
            rospy.loginfo("The rectangle is small. Use 2 pass")
            

        elif (self.rect.size.width < self.Effective_width):
            ### The rectangle is too small for one pass
            rospy.logwarn("The rectangle is too small")
        else :
            ### Something went wrong
            rospy.logwarn("Something went wrong with rectangle size")

         ### Separer le point cloud pour les points dans le rectangle
         # Example : pc[1:4][1:4]
         #box = cv2.boxPoints(self.right_rect_)
         

        self.points = []

        for rect in self.rects :

            point = Point()
            point.x = rect.x - math.sin(math.radians(rect.angle))*(rect.lenght-self.Effective_width)/2
            point.y = rect.y - math.cos(math.radians(rect.angle))*(rect.lenght-self.Effective_width)/2 
            self.points.append(point)
            point.x = rect.x + math.sin(math.radians(rect.angle))*(rect.lenght-self.Effective_width)/2
            point.y = rect.y + math.cos(math.radians(rect.angle))*(rect.lenght-self.Effective_width)/2
            self.points.append(point)
            point.x = point.x + math.sin(math.radians(rect.angle))*10
            point.y = point.y + math.sin(math.radians(rect.angle))*10
            self.points.append(point)

        ### Let's do orientation by localy checking the point cloud
        ### Make sure no point is out of the plaster by taking a small square

        self.pose_array_ = PoseArray()

        for point in self.points:
            self.PointToPose(point)


    def PointToPose(self,point):
        ### Take the point, tranform it to pose and append it to a PoseArray()
        pc_full = self.pc_raw
        pc = pc_full[..., :3]

        side = self.Effective_width*math.sqrt(2)/2
        side = np.int0(side)
        sub_pc = pc[point.x - side:point.x + side][point.y -side:point.y + side]

        plane = pyrsc.Plane()
        best_eq, best_inliers, perc_success, it_success = plane.fit(sub_pc, 0.003, minPoints=int(sub_pc.shape[0]*sub_pc.shape[1]*0.4), maxIteration=100)  # Good compromise
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


        ##self.EstimateXYresolution() ### Already done in rectangle segmentation

        self.pose = Pose()
        
        self.pose.position.x    = point.x * self.camera_resolutionX * 1000
        self.pose.position.y    = point.y * self.camera_resolutionY * 1000
        self.pose.position.z    = pc[np.int0(point.x)][np.int0(point.y)]
        self.pose.orientation.x = orientation_normalized[0]
        self.pose.orientation.y = orientation_normalized[1]
        self.pose.orientation.z = orientation_normalized[2]
        self.pose.orientation.w = orientation_normalized[3]

        self.pose_array_.header.frame_id = "camera"
        self.pose_array_.poses.append(self.pose)



    def PublishTraj(self):
        ### Publish the full trajectory msg to joint_path_command
        self.traj_pub.publish(self.full_trajectory)

    def PoseArrayToIKSrvCall(self):
        ### Take the PoseArray and call IK service for each PoseStamped
        
        ### Reset the trajectory
        self.full_trajectory = JointTrajectory()

        for pose in self.pose_array_:
            self.computeIKSrvCall(pose)



    def computeIKSrvCall(self,pose):
        ### PoseStamped() to joint_pose
        req = PositionIKRequest() ### Take only one PoseStamped() per request
        req.group_name = "manipulator"
        req.robot_state.joint_state = self.current_joint_pose
        req.ik_link_name = "TCP"
        req.pose_stamped = pose
        req.avoid_collisions = True
        
        #req.timeout = rospy.Duration(10)
        

        ### Ajoutons une joint constraint pour que l'axe du husky soit à zéro puisque la démo se fais sans cette axe.
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "husky_to_robot_base_plate"
        joint_constraint.position = 0.01
        joint_constraint.tolerance_below = 0.0
        joint_constraint.tolerance_above = 0.01
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


        rospy.loginfo(req)
        ### call le service
        res = self.compute_ik_client(req)
        ##rospy.loginfo(res.error_code.val)
        while not res.error_code.val == 1:
            rospy.logwarn("Did not find solution for IK. Looping back")
            res = self.compute_ik_client(req)
        
        ### Reponse du service call
        self.joint_pose = res
        
        ### Put joint_pose in a trajectory msg
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration(0.3)
        trajectory_point.positions = res.solution.joint_state.position
        self.full_trajectory.header = res.solution.joint_state.header
        ### WARNING: Make sure joint name are in the correct order depending if the state are from
        ##self.full_trajectory.joint_names = self.joint_pose.solution.joint_state.name
        self.full_trajectory.joint_names = self.current_joint_pose.name
        ##rospy.logwarn(self.full_trajectory.joint_names)
        self.full_trajectory.points.append(trajectory_point)

        ### update curent joint with last IK solution
        self.current_joint_pose.position = self.joint_pose.solution.joint_state.position


def main():

    traj_planner_node = traj_planner()

    rospy.spin()

if __name__ == "__main__":
    main()



        












