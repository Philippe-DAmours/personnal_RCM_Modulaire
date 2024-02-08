# Python program to inverse
# a matrix using numpy

# Import required package
import numpy as np
import os
import cv2
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

#os.chdir("/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/test_data/data_2023-03-14 09:43:40/")  # Only takeB
#os.chdir("/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/test_data/data_2023-03-10 13:30:34/")  # Only takeA

os.chdir("/home/introlab/Documents/git_rcm/rcm_poncage/pa_tof/test_data/data_2023-03-14 15:47:48/")  # Both takes

img = cv2.imread('img_raw1.png', cv2.IMREAD_UNCHANGED)
img_norm = cv2.normalize(img, None, 0, 65535, cv2.NORM_MINMAX, dtype=cv2.CV_16U)
plt.figure()
plt.imshow(img_norm)
plt.title("img")
plt.xlabel("width")
plt.ylabel("heigth")

border_y = int(img.shape[0]*0.3)
border_x = int(img.shape[1]*0.3)
img_scaled = img[border_y:img.shape[0]-border_y, border_x:img.shape[1]-border_x]

n = np.load("param_n.npy")
Ro = np.load("param_Ro.npy")

# Classification
data_conc = np.concatenate((n.flatten()[...,None], Ro.flatten()[...,None]), axis=1)
#data_centro = np.array([[]])
clf = KMeans(n_clusters=2)
results = clf.fit_predict(data_conc)
r_2d = results.reshape(n.shape)

n_class0 = np.where(r_2d==0, n, 0)
Ro_class0 = np.where(r_2d==0, Ro, 0)

n_class1 = np.where(r_2d==1, n, 0)
Ro_class1 = np.where(r_2d==1, Ro, 0)

plt.figure()
#plt.scatter(n.flatten(), Ro.flatten(), marker=',', color='b')
plt.scatter(n_class0.flatten(), Ro_class0.flatten(), marker=',', color='b')
plt.scatter(n_class1.flatten(), Ro_class1.flatten(), marker=',', color='r')
plt.title("Surface physic parameters")
plt.xlabel("n")
plt.ylabel("Ro")
plt.axis([0, 2, 0, 2])

plt.figure()
plt.imshow(Ro)
plt.title("Ro")
plt.xlabel("width")
plt.ylabel("heigth")

plt.figure()
plt.imshow(n)
plt.title("n")
plt.xlabel("width")
plt.ylabel("heigth")

k = 5  # arbitrary values from article
kernel_size = (k,k)

filtered = cv2.medianBlur(np.float32(r_2d), k)
plt.figure()
plt.imshow(filtered)
plt.title("filtered")
plt.xlabel("width")
plt.ylabel("heigth")

Ro_filtered = cv2.medianBlur(np.float32(Ro_class1), k)
plt.figure()
plt.imshow(Ro_filtered)
plt.title("Ro_filtered")
plt.xlabel("width")
plt.ylabel("heigth")

n_filtered = cv2.medianBlur(np.float32(n_class1), k)
plt.figure()
plt.imshow(n_filtered)
plt.title("n_filtered")
plt.xlabel("width")
plt.ylabel("heigth")

plt.figure()
plt.imshow(img_scaled)
plt.title("img_scaled")
plt.xlabel("width")
plt.ylabel("heigth")

plt.show()
