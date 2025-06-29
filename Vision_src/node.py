#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt


class TestScript(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info("Test node for testing purposes :3")
        hz = 2 #period = 1/2 = 0.5s = 500ms
        timer_ = self.create_timer(1/hz, self.timer_callback) 

    def timer_callback(self):
        self.get_logger().info("Hello from node")

    def segment_and_find_longest(image_path):
        if(image_path =="blank"): return
        image = cv2.imread(image_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)
    
        # Morphological opening to separate parts (adjust kernel size for better splitting)
        kernel = np.ones((30, 50), np.uint8)
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    
        # Find connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(opened, connectivity=8)
    
        result_image = image.copy()
        max_length = 0
        longest_center = (0, 0)
    
        for i in range(1, num_labels):  # Skip label 0 (background)
            x, y, w, h, area = stats[i]
            cx, cy = centroids[i]
    
            # Draw red box around each component
            cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
    
            # Use diagonal length as a proxy for segment size
            length = np.sqrt(w**2 + h**2)
            if length > max_length:
                max_length = length
                longest_center = (int(cx), int(cy))
    
        # Mark the center of the longest segment
        cv2.circle(result_image, longest_center, 7, (255, 0, 0), -1)
    
        # Display result
        plt.figure(figsize=(8, 8))
        plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
        plt.title(f"Longest Segment Center: {longest_center}")
        plt.axis('off')
        plt.show()
    
        return longest_center

def main(args=None):
    rclpy.init(args=args)
    node = TestScript()
    node.add_on_set_parameters_callback("blank")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()