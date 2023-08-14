#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import os
import sys
import termios
import tty
import tf2_ros as tf2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Get keys and save by hand.
# Depricated -> We now save by right clicking the desired image

def getKey(settings):
  tty.setraw(sys.stdin.fileno())
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

def saveTerminalSettings():
  return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


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


def drawBox(image):
  _, contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  img_box = image.copy()

  area_sander = 6000   # pixels -> TODO: ajuster
  min_side_size = 50  # pixels
  dim_array = np.array([[0,0]])
  multi_points_array = np.array([[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]])
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
          img_box = cv2.drawContours(image,[box],0,(255,0,0),2)
      
          dim_array = np.append(dim_array, [[dim1,dim2]], axis=0)
          if(dim1 * dim2 > area_sander):
              if(dim1 > dim2):
                  mid_point_1_x = (box[0,0]+box[1,0])/2
                  mid_point_1_y = (box[0,1]+box[1,1])/2
                  mid_point_2_x = (box[2,0]+box[3,0])/2
                  mid_point_2_y = (box[2,1]+box[3,1])/2
                  multi_points_array = np.append(multi_points_array, [[[box[0,0], box[0,1]], 
                                                                      [mid_point_1_x, mid_point_1_y],
                                                                      [box[1,0], box[1,1]], 
                                                                      [box[2,0], box[2,1]], 
                                                                      [mid_point_2_x, mid_point_2_y],
                                                                      [box[3,0], box[3,1]]]], axis=0)
              else:
                  mid_point_1_x = (box[1,0]+box[2,0])/2
                  mid_point_1_y = (box[1,1]+box[2,1])/2
                  mid_point_2_x = (box[3,0]+box[0,0])/2
                  mid_point_2_y = (box[3,1]+box[0,1])/2
                  multi_points_array = np.append(multi_points_array, [[[box[0,0], box[0,1]], 
                                                                      [box[1,0], box[1,1]], 
                                                                      [mid_point_1_x, mid_point_1_y],
                                                                      [box[2,0], box[2,1]], 
                                                                      [box[3,0], box[3,1]],
                                                                      [mid_point_2_x, mid_point_2_y]]], axis=0)
                  
          else:
              single_point_array = np.append(single_point_array, [[(box[0,0]+box[2,0])/2, 
                                                                  (box[0,1]+box[2,1])/2]], axis=0)              

  dim_array = np.delete(dim_array, 0, 0)   #Remove first all zeros value
  multi_points_array = np.delete(multi_points_array, 0, 0)   #Remove first all zeros value
  single_point_array = np.delete(single_point_array, 0, 0)   #Remove first all zeros value
  # print("Dimension de chaques rectangles: \n", dim_array)
  # print("6 points contours de chaques grands rectangles: \n", multi_points_array)
  # print("Point central de chaques petits rectangles: \n", single_point_array)

  return img_box, dim_array, multi_points_array, single_point_array


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_2ponce", Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    self.last_cv_image_raw = np.zeros((100,100,3), dtype=np.uint8)
    self.last_cv_image_gray = np.zeros((100,100,3), dtype=np.uint8)
    self.last_cv_image_clahe = np.zeros((100,100,3), dtype=np.uint8)
    self.last_cv_image_rosin = np.zeros((100,100,3), dtype=np.uint8)
    self.last_cv_image_scanned = np.zeros((100,100,3), dtype=np.uint8)
    self.img_index = 0

  def callback(self,data):
    try:
      self.last_cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.last_cv_image_scanned = self.last_cv_image_raw.copy()
    except CvBridgeError as e:
      print(e)

    self.last_cv_image_gray = cv2.cvtColor(self.last_cv_image_raw, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=6.0, tileGridSize=(8, 8))
    self.last_cv_image_clahe = clahe.apply(self.last_cv_image_gray)
    self.last_cv_image_rosin = rosinThreshold(self.last_cv_image_clahe)
    self.last_cv_image_scanned, dim_array, multi_points_array, single_point_array = drawBox(self.last_cv_image_rosin)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.last_cv_image_scanned, "mono8"))
    except CvBridgeError as e:
      print(e)

  def save_last_image(self):
    #print("We are in")
    path = "/home/introlab/Documents/git_RCM/rcm_poncage/pa_rgb/images"
    os.chdir(path)
    cv2.imwrite(("img_raw_" + str(self.img_index) + ".jpg"), self.last_cv_image_raw)
    cv2.imwrite(("img_gray_" + str(self.img_index) + ".jpg"), self.last_cv_image_gray)
    cv2.imwrite(("img_clahe_" + str(self.img_index) + ".jpg"), self.last_cv_image_clahe)
    cv2.imwrite(("img_scan" + str(self.img_index) + ".jpg"), self.last_cv_image_rosin)
    cv2.imwrite(("img_scan" + str(self.img_index) + ".jpg"), self.last_cv_image_scanned)
    #print("Last image saved! -> ", self.img_index)

    self.img_index += 1
    

def main(args):
  settings = saveTerminalSettings()

  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  r = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():
    r.sleep()

    key = getKey(settings)
    #ascii_char = [ord(chars) for chars in key]
    #print("key:", ascii_char)
    #print("Comparaison: ", ord('p') )
    if key == 'p':
      ic.save_last_image()
   
  
  cv2.destroyAllWindows()
  restoreTerminalSettings(settings)

if __name__ == '__main__':
    main(sys.argv)