# Strongly inspired from: https://github.com/leomariga/pyRANSAC-3D
# Which is under Apache 2.0 license

import random

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

        return self.equation, self.inliers, perc_success, it
