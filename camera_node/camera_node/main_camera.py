import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
from math import sqrt, pi, radians, cos, sin

angle, true_length_mm, center = 0
quat_z = [1,0,0,0]
# Open the default camera (index 0)
class CameraDetection():
        
    def Initialization():
    
        cam = cv.VideoCapture(0)

        # Initialize flags and variables
        paper_detected = False       # True once paper is found
        paper_roi = None             # Region of interest of the paper (x, y, width, height)
        background_roi = None        # Background image of paper to compare for changes
        tracking_started = False     # True once object tracking is active

        # Known physical height of folded A4 paper in millimeters (half of 297 mm)
        paper_height_mm = 297 / 2

        while True:
            ret, frame = cam.read()  # Capture a frame from the camera
            if not ret:
                break               # Exit loop if frame is not captured correctly

            im = frame.copy()       # Copy frame for drawing and processing
            combined = im           # Used later for combined display

            # Step 1: Detect paper if not already detected
            if not paper_detected:
                gray_im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)  # Convert to grayscale for thresholding
                mask = cv.inRange(gray_im, 150, 255)          # Threshold to isolate bright regions (paper)
                kernel = np.ones((5, 5), np.uint8)
                mask = cv.erode(mask, kernel, iterations=2)   # Erode mask to reduce noise

                # Find contours of white regions in mask
                contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                # Filter contours to keep only large ones (likely paper)
                large_contours = [cnt for cnt in contours if cv.contourArea(cnt) > 10000]

                if large_contours:
                    # Pick the largest contour as the paper
                    cnt = max(large_contours, key=cv.contourArea)
                    epsilon = 0.02 * cv.arcLength(cnt, True)
                    approx = cv.approxPolyDP(cnt, epsilon, True)  # Approximate polygon for contour
                    x, y, w, h = cv.boundingRect(approx)           # Bounding rectangle for the paper
                    paper_roi = (x, y, w, h)
                    # Draw green rectangle around detected paper
                    cv.rectangle(im, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Step 2: If paper detected, highlight and start tracking objects on it
            if paper_detected:
                x, y, w, h = paper_roi
                # Draw rectangle around paper for visual feedback
                cv.rectangle(im, (x, y), (x+w, y+h), (0, 255, 0), 2)

                if tracking_started:
                    # Extract grayscale region of paper for current frame
                    current_roi = cv.cvtColor(frame[y:y+h, x:x+w], cv.COLOR_BGR2GRAY)
                    # Compute difference from locked background (paper without objects)
                    diff = cv.absdiff(background_roi, current_roi)
                    # Threshold difference to get binary mask of new objects
                    _, thresh = cv.threshold(diff, 30, 255, cv.THRESH_BINARY)

                    # Clean up mask using morphological operations
                    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
                    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)   # Remove small noise
                    thresh = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)  # Fill gaps
                    thresh = cv.GaussianBlur(thresh, (5, 5), 0)               # Smooth edges
                    _, thresh = cv.threshold(thresh, 50, 255, cv.THRESH_BINARY)

                    # Find contours of objects on paper
                    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                    # Process each contour (candidate object)
                    for i, c in enumerate(contours):
                        area = cv.contourArea(c)
                        # Filter out too small or too large objects
                        if area < 3700 or area > 100000:
                            continue

                        # Get minimum-area rotated rectangle around contour
                        rect = cv.minAreaRect(c)
                        box = cv.boxPoints(rect).astype(int)
                        # Adjust box points to frame coordinates (relative to full image)
                        box[:, 0] += x
                        box[:, 1] += y

                        # Extract center coordinates, width, height, and rotation angle
                        center = (int(rect[0][0] + x), int(rect[0][1] + y))
                        width = rect[1][0]
                        height = rect[1][1]
                        angle = rect[2]
                        # Adjust angle so it represents object rotation correctly
                        if width < height:
                            angle = 90 - angle
                        else:
                            angle = -angle

                        # Draw red contour box and green center dot on image
                        cv.drawContours(im, [box], 0, (0, 0, 255), 2)
                        cv.circle(im, center, 4, (0, 255, 0), -1)

                        # Find longest edge of the bounding box
                        max_dist = 0
                        pt1, pt2 = None, None
                        for i in range(4):
                            dx = box[i][0] - box[(i+1)%4][0]
                            dy = box[i][1] - box[(i+1)%4][1]
                            dist = dx*dx + dy*dy
                            if dist > max_dist:
                                max_dist = dist
                                pt1 = box[i]
                                pt2 = box[(i+1)%4]

                        # Vector along the longest edge
                        line_dx = pt2[0] - pt1[0]
                        line_dy = pt2[1] - pt1[1]
                        length = sqrt(line_dx**2 + line_dy**2)

                        # Calculate perpendicular (normal) direction to that edge
                        normal_dx = -line_dy / length
                        normal_dy = line_dx / length
                        direction = np.array([normal_dx, normal_dy])

                        # Calculate intersections of the perpendicular line through center with the box edges
                        intersections = []
                        for i in range(4):
                            p1 = box[i]
                            p2 = box[(i+1)%4]
                            edge_vec = np.array(p2) - np.array(p1)
                            edge_perp = np.array([-edge_vec[1], edge_vec[0]])
                            denom = np.dot(edge_perp, direction)
                            if abs(denom) < 1e-6:
                                continue
                            t = np.dot(edge_perp, np.array(p1) - np.array(center)) / denom
                            intersect = np.array(center) + t * direction
                            dot1 = np.dot(intersect - p1, edge_vec)
                            dot2 = np.dot(intersect - p2, -edge_vec)
                            # Check if intersection lies on the edge segment
                            if dot1 >= 0 and dot2 >= 0:
                                intersections.append(tuple(intersect.astype(int)))

                        # Calculate true thickness as distance between intersections on box edges
                        if len(intersections) == 2:
                            cv.line(im, intersections[0], intersections[1], (255, 0, 255), 2)
                            true_length_px = sqrt((intersections[0][0] - intersections[1][0])**2 +
                                                  (intersections[0][1] - intersections[1][1])**2)
                        else:
                            true_length_px = length  # fallback to longest edge length

                        # Convert pixel length to millimeters using known paper height
                        pixels_per_mm = max(w, h) / paper_height_mm
                        true_length_mm = true_length_px / pixels_per_mm

                        # Calculate center position relative to camera center in mm
                        rel_center_x = center[0] - frame.shape[1]/2
                        rel_center_y = center[1] - frame.shape[0]/2
                        center_mm = (rel_center_x / pixels_per_mm, rel_center_y / pixels_per_mm)

                        # Display detected information near the object
                        info_lines = [
                            f"Center px: ({center[0]}, {center[1]})",
                            f"Center mm: ({center_mm[0]:.1f}, {center_mm[1]:.1f})",
                            f"Rotation: {angle:.1f} deg",
                            f"grip_distance: {true_length_mm:.1f} mm"
                        ]

                        for i, text in enumerate(info_lines):
                            cv.putText(im, text, (center[0] - 100, center[1] + 20 + i*25),
                                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                        # Convert angle to radians
                        theta_rad = radians(angle)  # angle is in degrees

                        # Compute quaternion for Z-axis rotation
                        # Only Z rotation → (w, x, y, z)
                        quat_z = (
                            cos(theta_rad / 2),  # w
                            0.0,                 # x
                            0.0,                 # y
                            sin(theta_rad / 2)   # z
                        )
                        print(quat_z)

                    # Show binary mask next to original image for debugging
                    thresh_bgr = cv.cvtColor(thresh, cv.COLOR_GRAY2BGR)
                    thresh_resized = cv.resize(thresh_bgr, (im.shape[1], im.shape[0]))
                    combined = np.hstack((im, thresh_resized))
                else:
                    # Prompt user to lock background before tracking
                    cv.putText(im, "Press T to lock background", (10, 30),
                                cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    combined = im

            else:
                cv.putText(im, "Press S to lock workspace", (10, 30),
                                cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                combined = im

            # Display the combined frame
            cv.imshow("Live Feed + Threshold + test", combined)

            key = cv.waitKey(30) & 0xFF
            if key == 27:  # ESC key to quit
                break
            elif key == ord('t') and paper_detected:
                # Lock current paper image as background for tracking objects
                background_roi = cv.cvtColor(frame[y:y+h, x:x+w], cv.COLOR_BGR2GRAY)
                tracking_started = True
                print("Background locked. Tracking started.")
            elif key == ord('s') and not paper_detected:
                # Mark paper as detected manually
                paper_detected = True

        cam.release()
        cv.destroyAllWindows()
    def get_data():
        return [center_mm[0], center_mm[1],0, quat_z[0], quat_z[1],quat_z[2], quat_z[3] ]

    def test_print(): 
        return "This is a test message"
