# Enhanced Point Cloud Based 3D City Reconstruction from Google Street View
# Author: Jinxuan Liang Yihua Zhou

# Usage

## Data Collection ( currently only safari is supported)
Go to data collection directory and open example.html file\
Input the longitude and latitude that you are interested in to the line below.\
panoLoader.load(new google.maps.LatLng(42.345601, -71.098348));\
An image and a csv file containing depth data will be downloaded automatically.

## Reconstruction
We provide a demo notebook under point cloud directory to reconstruction union square in san francisco.\
You can also use the same notebook with you own collected data.\
Before you run the notebook, remember to change the center latitude, longitude and heading that you select.

## Video Demo
### Click to play our video demo
[![Video Title](https://img.youtube.com/vi/TUSRpaOvvdU/0.jpg)](https://youtu.be/TUSRpaOvvdU)

## Reconstruction results
![demo1](https://github.com/RichZhou1999/street_view_project/assets/91929958/9baff3af-5b28-49c5-a587-bf26c07d42a3)
Tower bridge in London
![demo4](https://github.com/RichZhou1999/street_view_project/assets/91929958/9287c43f-6821-44cb-a10a-d40cecff3258)
Times square in NY
![demo3](https://github.com/RichZhou1999/street_view_project/assets/91929958/0bab5f73-9aef-4cad-870e-f53b7dc2318c)
Times square from a far view

## A large scenario
Download the file from https://drive.google.com/file/d/1AaIaeoWAsG6MYssynZZ-oJSa5NcoqX8R/view?usp=sharing \
You can run the notebook inside the file and reconstruct times square.

### Part of the code is from https://github.com/proog128/GSVPanoDepth.js
