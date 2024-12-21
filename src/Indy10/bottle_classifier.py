#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from indy_driver.msg import warning  # 메시지 타입 변경
from cv_bridge import CvBridge, CvBridgeError

# Define ROIs for 4 bottles (adjust coordinates as needed)
bottle_rois = [
    (86, 231, 134, 476),   # Bottle 1 ROI
    (219, 229, 303, 478), # Bottle 2 ROI
    (342, 231, 434, 476), # Bottle 3 ROI
    (489, 233, 585, 474)  # Bottle 4 ROI
]

# Thresholds for warnings
LOW_THRESHOLD = 20  # Warning when liquid is below 20%
EMPTY_THRESHOLD = 5  # Almost empty when below 5%

class BottleLiquidDetectionNode:
    def __init__(self):
        rospy.init_node('bottle_liquid_detection', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw2", Image, self.image_callback)
        self.warning_pub = rospy.Publisher("warning", warning, queue_size=1)  # 메시지 타입을 String으로 변경
 
        # HSV ranges for each bottle (default values)
        self.hsv_lower = [
            np.array([20, 80, 10 ]), # Bottle 1
            np.array([10, 150, 100]), # Bottle 2
            np.array([0, 74, 10]), # Bottle 3
            np.array([10, 87, 114])  # Bottle 4
        ]
        self.hsv_upper = [
            np.array([60, 255, 255]), # Bottle 1
            np.array([25, 255, 255]), # Bottle 2
            np.array([180, 255, 100]), # Bottle 3
            np.array([26, 255, 255])  # Bottle 4
        ]

        self.status_counter = 0
        self.previous_status = None  # Last published status

    def calculate_liquid_level(self, frame, roi, lower_color, upper_color):
        x1, y1, x2, y2 = roi
        roi_frame = frame[y1:y2, x1:x2]

        # Convert ROI to HSV for color segmentation
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the liquid using the provided HSV range
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Calculate the percentage of liquid pixels
        total_pixels = mask.size
        liquid_pixels = cv2.countNonZero(mask)
        percentage = (liquid_pixels / total_pixels) * 100

        return percentage

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Calculate the current status (0~4)
        status = 0
        for i, roi in enumerate(bottle_rois):
            liquid_percentage = self.calculate_liquid_level(frame, roi, self.hsv_lower[i], self.hsv_upper[i])
            if liquid_percentage < EMPTY_THRESHOLD:
                status = max(status, i + 1)

            # Draw ROI and bottle number
            x1, y1, x2, y2 = roi
            color = (0, 255, 0) if liquid_percentage >= LOW_THRESHOLD else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, str(i + 1), (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        # Update status tracking
        if self.previous_status == status:
            self.status_counter += 1
        else:
            self.status_counter = 0
        self.previous_status = status

        # If the status remains unchanged for 30 frames, publish the warning as a string
        if self.status_counter >= 30:
            self.warning_pub.publish(str(status))  # 문자열로 변환하여 퍼블리시
            rospy.loginfo(f"Warning Published: {status}")
            self.status_counter = 0  # Reset counter after publishing

        # Display the annotated frame
        cv2.imshow('Bottle Liquid Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User Exit')

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = BottleLiquidDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass 