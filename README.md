# Development-of-Social-Distancing-Alert-System
This project uses Homography transform and OpenVINO Pedestrian Tracker Demo. Homography transform will be used to map image/video points to its corresponding real world points on the same image/video.

# Project Purpose
The main purpose of this system is to draw the position of people seen in video into a map, the distance between each person is measured on the map based on points. Sound alarm if distance between points are less than 1 meter. 

# Project Development Stages
This project is still under development, Stage 4.

Stage 1 - Put input and Homography Transform of video/image
Stage 2 - Get human position (coordinates) and transform them into real world points
Stage 3 - Draw transformed points on map
Stage 4 - Measure distance between points
Stage 5 - Sound alarm if distance less than 1 meter

# (Credits)
Homography Code (Courtesy of Mika)
https://stackoverflow.com/questions/25769707/camera-pixels-to-planar-world-points-given-4-known-points

getPerspectiveTransform Source Code (Courtesy of OpenCV Dev)
https://github.com/opencv/opencv/blob/master/modules/imgproc/src/imgwarp.cpp
