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
import skimage.transform as sktransform

csv_name = 'example.csv'

data = np.genfromtxt(csv_name, delimiter=',')

def truncate_values(array):
    for i in range(len(array)):
        for j in range(len(array[i])):
            if array[i][j] > 255:
                array[i][j] = 255
    return array
data = truncate_values(data)

print(data)
skio.imsave('depth.png', data)
# Display the data
print(data.shape)

pic_name = "depth.png"
im = skio.imread(pic_name)
print(im)