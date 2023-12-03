import numpy as np
import skimage as sk
import skimage.io as skio
import json
import open3d as o3d
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
import numpy as np

h = 256
w = 512

def calculate_cart_coordinate(depth):
    points = []
    selected_index = []
    for i in range(h):
        for j in range(w):
            phi = (w-j-1)/(w-1) * 2* 3.14159 + 0.5*3.14159
            theta = (h-i-1)/(h-1) * 3.14159
            
#             phi = (w-i)/(w) * 2* 3.14159
#             theta = (1-(h-j)/(h)) * 3.14159
            v = np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)])
#             points.append(copy.copy(v*depth[j,i]*1000))
            if depth[i,j] != 1:
                points.append(copy.copy(v*depth[i,j]*255))
                
                selected_index.append(i*w + j)
    return points, selected_index
            
def calculate_cart_coordinate_populate(depth):
    points = []
    selected_index = []
    for i in range(h):
        for j in range(w):
            for offset_i in np.linspace(0, 1, 5):
                for offset_j in np.linspace(0, 1, 5):
                    phi = (w-(j+offset_j)-1)/(w-1) * 2* 3.14159 + 0.5*3.14159
                    theta = (h-(i+offset_i)-1)/(h-1) * 3.14159
                    
        #             phi = (w-i)/(w) * 2* 3.14159
        #             theta = (1-(h-j)/(h)) * 3.14159
                    v = np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)])
        #             points.append(copy.copy(v*depth[j,i]*1000))
                    if depth[i,j] != 1:
                        points.append(copy.copy(v*depth[i,j]*255))
                        
                        selected_index.append(i*w + j)
    return points, selected_index

if __name__ == "__main__":
    depth = skio.imread("example_depth.png")
  
    depth = sk.img_as_float(depth)[:,:,0]
    colors = skio.imread("example_img.png")
  
    colors = sk.img_as_float(colors)
    colors = np.array(colors[:,:,:3])
    
    points, selected_index = calculate_cart_coordinate(depth)
    colors = colors.reshape(-1,3)
    colors = colors[selected_index]
    o3d.visualization.webrtc_server.enable_webrtc()
    
    # Define a list of points (replace this with your own coordinates)
    points = np.array(points)
    # Create a point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    larger_radii = 0.02 * np.ones(len(points))  # Set larger radii
    point_cloud.radius = o3d.utility.DoubleVector(larger_radii)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])