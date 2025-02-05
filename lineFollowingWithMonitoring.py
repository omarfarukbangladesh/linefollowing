#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np
from picamera2 import Picamera2, Preview
import time
import math
from YB_Pcb_Car import YB_Pcb_Car

# Initialize the car and camera
car = YB_Pcb_Car()
picam2 = Picamera2()

# Configure and start the camera with a preview (real-time monitoring)
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.start()

def find_white_line(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply a binary threshold to get a black and white effect. Adjust the threshold as needed.
    _, threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def find_blue_line(frame):
    # Convert the image from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of blue color in HSV
    lower_blue = np.array([100, 150, 150])  # Adjust these values as needed
    upper_blue = np.array([140, 255, 255])  # Adjust these values as needed

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours


try:
    while True:
        frame = picam2.capture_array()
        contours = find_white_line(frame)

        if contours:
            # Find the largest contour and its centroid
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Assuming the frame is 640x480, adjust these values accordingly
                frame_center = 320
                center_deviation = cx - frame_center
                
                # Adjust speeds based on the deviation
                base_speed = 60
                if abs(center_deviation) < 50:  # Go straight
                    car.Car_Run(base_speed, base_speed)
                elif center_deviation > 0:  # Turn right
                    car.Car_Run(base_speed, base_speed - min(abs(center_deviation) * 2, base_speed))
                else:  # Turn left
                    car.Car_Run(base_speed - min(abs(center_deviation) * 2, base_speed), base_speed)
            else:
                # Stop the car if no centroid found
                car.Car_Stop()
        else:
            # Stop the car if no white line is detected
            car.Car_Stop()

        # Optional: Display the frame with detected contour for debugging
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    # Stop the car and camera when you press Ctrl+C
    car.Car_Stop()
    picam2.stop()
    cv2.destroyAllWindows()  # Close the OpenCV window
    print("Stopped by User")

