# Import libraries at the top of the script
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import sys
import os
# Change the working directory to directory of this .py file.
# Now, you can easily refer to e.g. images in the same folder.
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# cam = cv.VideoCapture(1)

# while True:

def show_image(im, titel):
    cv.namedWindow(titel, cv.WINDOW_NORMAL) # kan geen window die openstaat kleiner maken gebruik nieuwe window namen
    w = 370

    s = w / im.shape[1] 

    h = int(s * im.shape[0])
    cv.resizeWindow(titel, w, h)
    cv.imshow(titel, im)
    cv.waitKey(0)

def different_planes (im):
    red_plane_im = im[:, :, 2]
    green_plane_im = im[:, :, 1]
    blue_plane_im = im[:, :, 0]
    gray_im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    return np.concatenate((red_plane_im, green_plane_im, blue_plane_im, gray_im), axis=1)

im = cv.imread('test.jpg')
 
show_image(im, "begin")
# ret, frame = cam.read()
# height, width, _ = frame.shape

# show_image(different_planes(im), 'R, G, B & Gray')
gray_im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)

# plt.hist(gray_im.ravel(), 256, [0,256]); 
# plt.title('Histogram to get threshold between background and plate')
# plt.show()

backgrounds = cv.inRange(gray_im, 180, 250) 
rem_background_img = cv.bitwise_and(gray_im, gray_im, mask=backgrounds)
show_image(rem_background_img, "rem back")

kernel = np.ones((5, 5),np.uint8)
erosion = cv.erode(rem_background_img,kernel,iterations = 2)
show_image(erosion, "rem erode")

# im_with_back = cv.bitwise_and(im, im, mask=erosion)
# show_image(im_with_back, 'image without background')

# Find contours
contours, _ = cv.findContours(erosion, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Filter out small contours
min_area = 10000  # adjust this threshold if needed
large_contours = [cnt for cnt in contours if cv.contourArea(cnt) > min_area]

# Draw contours
output = im.copy()
for cnt in large_contours:
    epsilon = 0.02 * cv.arcLength(cnt, True)
    approx = cv.approxPolyDP(cnt, epsilon, True)
    min_x = np.min(approx[:, 0, 0])
    min_y = np.min(approx[:, 0, 1])
    print(f"Top-left corner: x = {min_x}, y = {min_y}")
    min_x = np.max(approx[:, 0, 0])
    min_y = np.max(approx[:, 0, 1])
    print(f"right-bottom corner: x = {min_x}, y = {min_y}")
    cv.drawContours(output, [approx], -1, (0, 255, 0), 2)

show_image(output, "Detected square")



# show_image('removed back', rem_background_img)

# erosion = cv.erode(rem_background_img ,kernel,iterations = 1) 
# show_image('picture erosed', erosion)


    # contours, _ = cv.findContours(cam, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
   
    # for cnt in contours:
    #     # Calculate area and remove small elements
    #     area = cv.contourArea(cnt)
        
    #     if area > 100:
    #         cv.drawContours((height, width), [cnt], -1, (0, 255, 0), 2)
    #         x, y, w, h = cv.boundingRect(cnt)
    #         cv.rectangle((width, height), (x, y), (x + w, y + h), (0, 255, 0), 3)