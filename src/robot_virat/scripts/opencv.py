#!/usr/bin/env python3

import rospy
import cv2
import struct
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

header = Header()
header.frame_id = "body"


class camera_1:
    def __init__(self):
        # subscribing to the image data from the virat camera
        self.image_sub = rospy.Subscriber(
            "robot_virat/camera1/image_raw", Image, self.callback
        )
        self.pub = rospy.Publisher(
            "robot_virat/PointCloud", PointCloud2, queue_size=3
        )

    def callback(self, data):

        bridge = CvBridge()

        try:
            # using CVbridge to convert a ros image to a cv2 image
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)

        # masking the video
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, gray = cv2.threshold(gray, 127, 255, 0)
        gray2 = gray.copy()
        mask = np.zeros(gray.shape, np.uint8)
        contours, hier = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        points = []
        # checking for contours in the masked video
        for cnt in contours:
            if 100 < cv2.contourArea(cnt) < 50000:
                # drawing the potholes
                cv2.drawContours(cv_image, [cnt], -1, (0, 0, 255), 3)  # colored red
                cv2.drawContours(mask, [cnt], 0, 255, -1)
                # making a list of (x,y) points in contour to generate a pointcloud
                for c in cnt:
                    for p in c:
                        # apending the contour points to list 'points'
                        points.append((p[0] / 50, p[1] / 50, 0, 1))
                if cv2.contourArea(cnt) < 50000:
                    continue
        cv2.imshow("feed", cv_image)

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 16, PointField.UINT32, 1),
        ]

        header.stamp = rospy.Time.now()
        # generating the pointcloud
        cloud = point_cloud2.create_cloud(header, fields, points)
        # publishing the pointcloud to the 'PointCloud' topic
        self.pub.publish(cloud)

        cv2.waitKey(3)


# main function that calls on the class camera_1
def main():
    camera_1()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("virat_img_processor")
    main()
