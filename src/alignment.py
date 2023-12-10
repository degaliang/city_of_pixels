import numpy as np
import skimage as sk
import skimage.io as skio
import json
import copy
import triangle
import matplotlib.pyplot as plt
import skimage as ski
from PIL import Image
import imageio
import os
from tqdm import tqdm
import cv2
from scipy import signal
import copy
import open3d as o3d

# global variables for coordinate based alignment
global center_lat, center_lon, center_heading

center_lat = 37.78830013768426
center_lon = -122.40846027989743
center_heading = 170.830

# The following are coordinate based alignment functions
from math import radians, sin, cos, sqrt, atan2

def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    radius_of_earth = 6371 * 1000  # Radius of Earth in kilometers. Use 3959 for miles.

    # Calculate the distance
    distance = radius_of_earth * c

    return distance

class Scene():
    def __init__(self, img_path, depth_path):
        data_array = np.genfromtxt(depth_path, delimiter=',', dtype=float)
        self.depth = data_array
        self.colors = skio.imread(img_path)
        self.colors = sk.img_as_float(self.colors)[:,:,:3]
        self.points = []
        self.w = self.colors.shape[1]
        self.h = self.colors.shape[0]
        self.selected_index  = []
        self.displacement = np.array([0,0,0])
        self.rotation_matrix = None
        img_path_split = img_path[:-4].split("_")
        self.lat = float(img_path_split[0])
        self.lon = float(img_path_split[1])
#         print(img_path_split)
        self.heading = float(img_path_split[2])
    
    def calculate_initial_compass_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculates the initial compass bearing between two points.

        Parameters:
        - lat1, lon1: Latitude and longitude of the starting point (in degrees)
        - lat2, lon2: Latitude and longitude of the destination point (in degrees)

        Returns:
        - Initial compass bearing in degrees (from 0 to 360)
        """

        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        # Calculate the differences in longitude
        d_lon = lon2 - lon1

        # Calculate the initial bearing using atan2
        x = np.sin(d_lon) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(d_lon))
        

        initial_bearing = atan2(x, y)

        # Convert the initial bearing from radians to degrees
        initial_bearing = initial_bearing/3.14159 * 180

        # Normalize the initial bearing to the range [0, 360)
        initial_bearing = (initial_bearing + 360) % 360

        return initial_bearing
    
    
    def calcualte_displacement(self, lat1=center_lat, lon1=center_lon,heading1=center_heading):
            lat2 = self.lat
            lon2 = self.lon
            heading2 = self.heading
            
            
            dheading = self.calculate_initial_compass_bearing(lat1, lon1, lat2, lon2)
            
            # Convert latitude and longitude from degrees to radians
            lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

            # Haversine formula
            dlat = lat2 - lat1
            dlon = lon2 - lon1
            a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
            c = 2 * atan2(sqrt(a), sqrt(1 - a))
            radius_of_earth = 6371 * 1000  # Radius of Earth in kilometers. Use 3959 for miles.

            # Calculate the distance
            distance = radius_of_earth * c
            print('distance', distance)

            
            print('dheading', dheading)
            print('heading2', heading2)
            dheading = dheading/180 * 3.14159
            heading2 = heading2/180*3.14159

            
            rotation_matrix = np.array([[np.cos(heading2),-np.sin(heading2),0],
                           [np.sin(heading2),np.cos(heading2),0],
                           [0,0,1]])

            self.rotation_matrix = rotation_matrix
            self.displacement = [-np.cos(dheading)*distance, np.sin(dheading)*distance, 0]
    
    def calculate_xyz_coordinate(self):
        self.calcualte_displacement()
        total_count = 0
        for i in range(self.h):
            for j in range(self.w):
                if self.depth[i,j] < 1e7:
                    for offset_i in [0, 0.33,0.66]:
                        for offset_j in [0, 0.33,0.66]:
                            phi = (1-(self.w-(j+offset_j)-1)/(self.w-1)) * 2* 3.14159 - 3.14159
                            theta = (self.h-(i+offset_i)-1)/(self.h-1) * 3.14159


                            v = np.array([-np.sin(theta)*np.cos(phi), -np.sin(theta)*np.sin(phi), np.cos(theta)])

                            v = v @ self.rotation_matrix
                            self.points.append(copy.copy(v*self.depth[i,j]) + self.displacement)
                            self.selected_index.append(i*self.w + j)
                total_count+=1
    
    def calculate_xyz_coordinate(self):
        self.calcualte_displacement()
        total_count = 0
        for i in range(self.h):
            for j in range(self.w):
                if self.depth[i,j] < 1e7:
                    phi = (1-(self.w-j-1)/(self.w-1)) * 2* 3.14159 - 3.14159
                    theta = (self.h-i-1)/(self.h-1) * 3.14159

                    v = np.array([-np.sin(theta)*np.cos(phi), -np.sin(theta)*np.sin(phi), np.cos(theta)])

                    v = v @ self.rotation_matrix

                    self.points.append(copy.copy(v*self.depth[i,j]) + self.displacement)
                    self.selected_index.append(i*self.w + j)
                total_count+=1
  
# The followings are feature based alignment util functions

# adapted from https://github.com/nghiaho12/rigid_transform_3D
def rigid_transform(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # find rotation
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        # print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t.flatten()  

def calculate_cart_coordinate(depth, rgb):
    points = []
    colors = []
    h, w = depth.shape
    for i in range(h):
        for j in range(w):
            phi = (1-(w-j-1)/(w-1)) * 2* 3.14159 - 3.14159
            theta = (h-i-1)/(h-1) * 3.14159
            
            v = np.array([-np.sin(theta)*np.cos(phi), -np.sin(theta)*np.sin(phi), np.cos(theta)])

            if depth[i,j] < 1e7:
                points.append(copy.copy(v*depth[i,j]*255))
                colors.append(rgb[i, j])
                
    return np.array(points), np.array(colors)

def im2pts(depth, points_2d):
    points_3d = []
    h, w = depth.shape

    for i in range(points_2d.shape[0]):
        point = points_2d[i]
        r, c = point
        phi = (1-(w-c-1)/(w-1)) * 2* 3.14159 - 3.14159
        theta = (h-r-1)/(h-1) * 3.14159
        
        v = np.array([-np.sin(theta)*np.cos(phi), -np.sin(theta)*np.sin(phi), np.cos(theta)])

        i, j = int(r), int(c)
        if depth[i,j] < 1e7:
            points_3d.append(copy.copy(v*depth[i,j]*255))
        else:
            points_3d.append(np.zeros(3))
            
    return np.array(points_3d)

# RAndom SAmple Consensus(RANSAC)
def ransac(src_pts, ref_pts, eps=0.5, iters=100, sample_size=5):
    """
    This function uses RANSAC for estimating homography.
    It takes in two (n, 2) arrays of feature points. It
    returns the best least-squares H estimate that transforms 
    src_pts to ref_pts, along with the inliers that are used 
    to compute H as (n, 2) ndarrays in xy coordinates. 
    
    Args:
        src_pts (ndarray): (n, 3) array
        ref_pts (ndarray): (n, 3) array
        eps (float, optional): RANSAC threshold. Defaults to 0.5.
        iters (int, optional): number of RANSAC iterations to run. Defaults to 100.
    """
    

    num_pairs = src_pts.shape[0]
    src_inliers = np.empty((0, 2))
    ref_inliers = np.empty((0, 2))
    for _ in range(iters):
        sample_indices = np.random.choice(np.arange(num_pairs), sample_size, replace=False)
        src_samples = src_pts[sample_indices, :]
        ref_samples = ref_pts[sample_indices, :]
        
        R, t = rigid_transform(src_samples.T, ref_samples.T)
        
        pred_pts = src_pts @ R + t
        distances = np.linalg.norm(pred_pts - ref_pts, axis=1)
        inliers_indices = np.where(distances < eps)[0]
        
        if len(inliers_indices) > len(src_inliers):
            src_inliers = src_pts[inliers_indices]
            ref_inliers = ref_pts[inliers_indices]
    
    R, t = rigid_transform(src_samples.T, ref_samples.T)
    
    return R, t

# open3d helper functions for FPFH feature descriptors
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result  
        