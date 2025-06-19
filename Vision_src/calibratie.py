# Import libraries at the top of the script
import numpy as np
import cv2 as cv

import sys

import os
# Change the working directory to directory of this .py file.
# Now, you can easily refer to e.g. images in the same folder.
os.chdir(os.path.dirname(os.path.abspath(__file__)))

cam = cv.VideoCapture(0)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# img = cv.imread('checkerboard v2.jpg')

frame_width = int(cam.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv.CAP_PROP_FRAME_HEIGHT))

h = 370

s = h / frame_height

w = int(s * frame_width)

cv.namedWindow('gray checker', cv.WINDOW_NORMAL) 
cv.resizeWindow('gray checker', w, h)
# img_dark = cv.convertScaleAbs(cam, alpha=0.5, beta=0)

# gray = cv.cvtColor(cam, cv.COLOR_BGR2GRAY)
ret, frame = cam.read()
cv.imshow('gray checker', frame)
cv.waitKey(0)
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
ret, corners = cv.findChessboardCorners(gray, (7,9), None)
print("return", ret)

if ret == True:
    print('werkt')
    objpoints.append(objp)

    corners2 = cv.cornerSubPix(gray, corners, (7,9), (-1,1), criteria)
    imgpoints.append(corners2)

    cv.drawChessboardCorners(frame, (7,9), corners2, ret)
    cv.imshow('gray checker', frame)
    cv.waitKey()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (frame_width, frame_height), None, None)

    h,  w = frame_height, frame_width
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    print(newcameramtx)
    print("Frame size:", frame_width, "x", frame_height)
    print("Verwacht c_x:", frame_width / 2)
    print("Verwacht c_y:", frame_height / 2)
    undistorted = cv.undistortPoints(np.array([[[frame_width / 2, frame_height / 2]]], dtype=np.float32), mtx, dist, None, newcameramtx)
    print("Gecorrigeerde pixelpositie:", undistorted)


   