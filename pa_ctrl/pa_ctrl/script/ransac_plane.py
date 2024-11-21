# Strongly inspired from: https://github.com/leomariga/pyRANSAC-3D
# Which is under Apache 2.0 license

import random
import rospy
from datetime import datetime
import logging

import numpy as np


class Plane:
    """
    Implementation of planar RANSAC.

    Class for Plane object, which finds the equation of a infinite plane using RANSAC algorithim.

    Call `fit(.)` to randomly take 3 points of pointcloud to verify inliers based on a threshold.

    ![Plane](https://raw.githubusercontent.com/leomariga/pyRANSAC-3D/master/doc/plano.gif "Plane")

    ---
    """

    def __init__(self):
        self.inliers = []
        self.equation = []

    def fit_plane(self,point_cloud):
        ### Exemple from 
        ### https://programming-surgeon.com/en/fit-plane-python/
        """
        input
            point_cloud : list of xyz values numpy.array
        output
            plane_v : (normal vector of the best fit plane)
            com : center of mass
        """
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
    
    def custom_log(self,plane_v,perc_success,it):
        
        # Set up the logger
        logging.basicConfig(filename='/home/damp2404/catkin_ws/src/rcm_modulaire/pa_ctrl/pa_ctrl/script/ransac.log', level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        # Log a message with the current date and time
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        logging.info(f'Loged at {current_time}')

        logging.info(f'The reference  normal is: {plane_v}')
        logging.info(f'The bestRANSAC normal is: {self.equation}')
        logging.info(f'with percent succes {perc_success} and with {it} iterations')


    def ARCHIVEcustom_log(self,plane_v,perc_success,it):

        file = open("ransac.txt","w")
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        file.write(f'Loged at {current_time}')

        file.write(f'The reference  normal is: {plane_v}')
        file.write(f'The bestRANSAC normal is: {self.equation}')
        file.write(f'with percent succes {perc_success} and with {it} iterations')

        file.close()
        


    def fit(self, pts_matrix, thresh=0.05, minPoints=100, maxIteration=1000):
        """
        Find the best equation for a plane.

        :param pts: 3D point cloud as a `np.array (N,M,3)`.
        :param thresh: Threshold distance from the plane which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - `self.equation`:  Parameters of the plane using Ax+By+Cy+D `np.array (1, 4)`
        - `self.inliers`: points from the dataset considered inliers

        ---
        """
        # to get the points from different portion of the image ( |-- )
        #pts_list1 = pts_matrix[:, :int(pts_matrix.shape[1]/3), :].reshape(-1, pts_matrix.shape[-1])
        #pts_list2 = pts_matrix[:int(pts_matrix.shape[0]/2), int(pts_matrix.shape[1]/3)+1:, :].reshape(-1, pts_matrix.shape[-1])
        #pts_list3 = pts_matrix[int(pts_matrix.shape[0]/2)+1:, int(pts_matrix.shape[1]/3)+1:, :].reshape(-1, pts_matrix.shape[-1])
        #pts_listS = [pts_list1, pts_list2, pts_list3]

        pts_listALL = pts_matrix.reshape(-1, pts_matrix.shape[-1])  #Flattens the first 2 dimensions
        n_points = pts_listALL.shape[0]
        best_eq = []
        best_inliers = []

        if n_points == 0 :
            print("0 points given to RANSAC")
            return

        for it in range(maxIteration):

            # Samples 3 random points
            #pt_samples = np.zeros((3,3))
            #for i, list in enumerate(pts_listS):
            #    id_samples = random.sample(range(0, list.shape[0]), 1)
            #    pt_samples[i] = list[id_samples][0]

            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts_listALL[id_samples]

            # We have to find the plane equation described by those 3 points
            # We find first 2 vectors that are part of this plane
            # A = pt2 - pt1
            # B = pt3 - pt1

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            vecC = np.cross(vecA, vecB)

            # The plane equation will be vecC[0]*x + vecC[1]*y + vecC[0]*z = -k
            # We have to use a point to find k
            vecC = vecC / np.linalg.norm(vecC)
            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            # Distance from a point to a plane
            # https://mathworld.wolfram.com/Point-PlaneDistance.html
            pt_id_inliers = []  # list of inliers ids
            dist_pt = (
                plane_eq[0] * pts_listALL[:, 0] + plane_eq[1] * pts_listALL[:, 1] + plane_eq[2] * pts_listALL[:, 2] + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = plane_eq
                best_inliers = pt_id_inliers
            self.inliers = best_inliers
            self.equation = best_eq

            # Stop if threshold accuracy aquired
            if len(best_inliers) > minPoints:
                break
            
        # Print even if minPoints not reached
        perc_success = (len(best_inliers)/n_points)*100
        #print("RANSAC algo: ", perc_success, "% of the points in", it, " iterations")

        ref_normal = self.fit_plane(pts_listALL)

        self.custom_log(ref_normal,perc_success,it)

        return self.equation, self.inliers, perc_success, it
