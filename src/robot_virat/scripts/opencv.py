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
header.frame_id = "camera_link"


class camera_1:
    def __init__(self):
        # subscribing to the image data from the virat camera
        self.image_sub = rospy.Subscriber(
            "robot_virat/camera1/image_raw", Image, self.callback
        )
        self.pub = rospy.Publisher("robot_virat/PointCloud", PointCloud2, queue_size=3)
        self.cam_info = rospy.Subscriber(
            "/robot_virat/camera1/camera_info", CameraInfo, self.callback_2
        )

        self.KList = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    # subscribing to CameraInfo to obtain the intrinsic 'K' matrix, although it is in the form of the list right now
    def callback_2(self, msg):
        self.KList = msg.K

    # function that takes in pixels from contours and converts into distance with respect to the camera
    def pixels_to_meters(self, u, v):
        h = 1.18 #height of camera from the ground
        d = 0.73
        # converting the K list into a 3x3 matrix
        intrinsic_matrix = np.reshape(self.KList, (3, 3))

        yaw = 0
        # the camera tilts downward by a parameter of 0.45, setting pitch to -0.45 corrects it
        pitch = -0.45
        roll = 0
        field_of_view_deg = 45
        # finding inverse of intrinsic matrix
        inverse_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)

        #----------the rotation matrix-----------------------------------------------
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_ground_to_cam = np.array(
            [
                [cr * cy + sp * sr + sy, cr * sp * sy - cy * sr, -cp * sy],
                [cp * sr, cp * cr, sp],
                [cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy],
            ]
        )
        # finding the inverse of the rotation matrix, which is just its transpose
        rotation_cam_to_ground = rotation_ground_to_cam.T

        # finding the Xc,Yc.Zc coordinates wrt camera from the pixel coordinates (u,v)
        # The specific formula used is mentioned in the README file
        n = np.array([0, 1, 0])
        nc = (rotation_cam_to_ground.T).dot(n)

        uv = np.array([u, v, 1])
        Kinv_dot_uv = inverse_intrinsic_matrix.dot(uv)
        nc_dot_prod = nc.dot(Kinv_dot_uv)
        new_point = h * Kinv_dot_uv / nc_dot_prod
        return new_point

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
                        #processing the contour points using the function 'pixels_to_meters
                        new_p = self.pixels_to_meters(p[0], p[1])
                        # apending the processed contour points to list 'points', note the change in coordinate systems
                        points.append((new_p[2], -new_p[0], -new_p[1], 1))
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

