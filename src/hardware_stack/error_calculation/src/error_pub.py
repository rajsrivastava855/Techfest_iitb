#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rospy
from geometry_msgs.msg import Point  


rospy.init_node('error_publisher', anonymous=True)
error_pub = rospy.Publisher('/error_topic', Point, queue_size=10)

rate = rospy.Rate(10) 


video_capture = cv.VideoCapture(0)  # Change 0 to the video file path if needed


ret, frame = video_capture.read()
if not ret:
    print("Error: Unable to capture video.")
    exit(1)

(h, w) = frame.shape[:2]  # video frame dimensions
feed_center_x, feed_center_y = w // 2, h // 2

cv.namedWindow("Detected Color", cv.WINDOW_NORMAL)
cv.resizeWindow("Detected Color", 500, 500)

cv.namedWindow("Image Center", cv.WINDOW_NORMAL)
cv.resizeWindow("Image Center", 500, 500)

# Function: update HSV range using trackbars
def on_trackbar(val):
    pass  

# trackbars
cv.createTrackbar('H Lower', 'Detected Color', 0, 179, on_trackbar)
cv.createTrackbar('S Lower', 'Detected Color', 31, 255, on_trackbar)
cv.createTrackbar('V Lower', 'Detected Color', 0, 255, on_trackbar)
cv.createTrackbar('H Upper', 'Detected Color', 25, 179, on_trackbar)
cv.createTrackbar('S Upper', 'Detected Color', 255, 255, on_trackbar)
cv.createTrackbar('V Upper', 'Detected Color', 255, 255, on_trackbar)

while not rospy.is_shutdown():
    
    ret, frame = video_capture.read()
    if not ret:
        print("Error: Unable to capture frame. Exiting.")
        break

    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

   
    h_lower = cv.getTrackbarPos('H Lower', 'Detected Color')
    s_lower = cv.getTrackbarPos('S Lower', 'Detected Color')
    v_lower = cv.getTrackbarPos('V Lower', 'Detected Color')
    h_upper = cv.getTrackbarPos('H Upper', 'Detected Color')
    s_upper = cv.getTrackbarPos('S Upper', 'Detected Color')
    v_upper = cv.getTrackbarPos('V Upper', 'Detected Color')

    lower_range = np.array([h_lower, s_lower, v_lower], dtype="uint8")
    upper_range = np.array([h_upper, s_upper, v_upper], dtype="uint8")

    mask = cv.inRange(hsv_frame, lower_range, upper_range)

    detected_color = cv.bitwise_and(frame, frame, mask=mask)

    moments = cv.moments(mask)
    if moments["m00"] != 0:  
        cx = int(moments["m10"] / moments["m00"])  # Centroid X
        cy = int(moments["m01"] / moments["m00"])  # Centroid Y

        cv.circle(mask, (cx, cy), 20, (0, 0, 255), -1)

       
        error_x = (cx - feed_center_x)
        error_y = (cy - feed_center_y)

     
        print(f"Image Center: X={feed_center_x}, Y={feed_center_y}")
        print(f"Detected Center: X={cx}, Y={cy}")
        #print(f"Error in x : {error_x:.2f}")
        print(f"Error in y : {error_y:.2f}")

        error_msg = Point()
        error_msg.x = error_x
        error_msg.y = error_y
        error_msg.z = 0 
        error_pub.publish(error_msg)
    else:
        print("No object detected")

    
    feed_center_image = cv.circle(frame.copy(), (feed_center_x, feed_center_y), 15, (255, 255, 255), -1)
    cv.imshow("Image Center", feed_center_image)
    cv.imshow("Detected Color", mask)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

video_capture.release()
cv.destroyAllWindows()


