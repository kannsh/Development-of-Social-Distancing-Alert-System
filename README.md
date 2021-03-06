# MUST READ (Instructions)
OpenVINO 2018 R5 Toolkit must be installed to start this project. Refer to both Documentation pdf guides (Installation & Running demo)


OpenVINO 2018 R5 Toolkit Download Link - http://registrationcenter- download.intel.com/akdlm/irc_nas/15013/l_openvino_toolkit_p_2018.5.445.tgz

Read "Run pedestrian with homography guide" before running the C++ code


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

# File wiki
config.txt is for the user to key in the points used in video (4 imagepoints and its 4 corresponding worldpoints).

< homography.h >
contains the headers used and declared functions/classes which will be used:
Function SetupTransformationMatrix
Function TransformPoint
Class Obj P

< homography.cpp >
defines what happens inside the function and classes 

< main.cpp >
contains the OpenVINO Pedestrian Tracker Demo code and includes homography.h & homography.cpp 
The functions in homography.h and .cpp are called into the main.cpp file
The code inside main.cpp draws the bounding box of human detected, its position and coordinate values
The position of people are transformed into real world coordinates and drawn onto the map. 
The output in terminal are the real world (x,y) coordinates of human that were detected in video.

# (Credits)
Homography Code (Courtesy of Mika)
https://stackoverflow.com/questions/25769707/camera-pixels-to-planar-world-points-given-4-known-points

getPerspectiveTransform Source Code (Courtesy of OpenCV Dev)
https://github.com/opencv/opencv/blob/master/modules/imgproc/src/imgwarp.cpp

