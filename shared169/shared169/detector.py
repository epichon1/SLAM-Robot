#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Detect the ArUco marker with OpenCV.
#
#   Node:           /detectaruco
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /detectaruco/image_raw      Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, CameraInfo
# from roboheist_msgs.msg import Treasure
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R
from .utils import tf_inv, tf_mult
from pacman_msgs.msg import PointWithIDStamped


#
#  Detector Node Class
#
class DetectorNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image', 3)
        # self.pub_treas = self.create_publisher(Treasure, name+'/treasure', 10)
        self.pub_obs = self.create_publisher(PointWithIDStamped, '/aruco_obstacles', 10)
        self.pub_dots = self.create_publisher(PointWithIDStamped, '/aruco_dots', 10)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Set up the ArUco detector.  Use a dictionary with the
        # appropriate tags.  DICT_6x6_1000 has 1000 options (way too
        # many).  DICT_6x6_250 has 250 options.  DICT_6x6_50 is
        # probably enough for our projects...
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.params = cv2.aruco.DetectorParameters_create()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)
        
        self.camerainfo = CameraInfo()
        self.camerainfo.height = 480
        self.camerainfo.width = 640
        self.camerainfo.k = [678.7767, 0.0, 292.4464, 0.0, 688.32854, 225.39419, 0.0, 0.0, 1.0]
        self.camerainfo.d = [-0.0537811, 0.09419898, -0.00825154, -0.0088897, 0.0]
        self.camerainfo.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camerainfo.p = [674.73, 0.0, 287.4818, 0.0, 0.0, 686.00525, 221.852190, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.camD = self.camerainfo.d
        self.camK = self.camerainfo.k
        self.caminfoready = True
        self.info_sub = self.create_subscription(
            CameraInfo, '/usb_cam/camera_info', self.info_cb, 1)
            
        # Some fixed transforms for aruco tag positions on robot
        # 0: Front, 1: Right, 2: Back, 3: Left (assuming caster front)
        # self.aruco2base0 = Transform()
        # self.aruco2base0.translation.x = 0.16
        # self.aruco2base0.translation.y = 0.0
        # self.aruco2base0.translation.z = 0.06
        # quat0 = R.from_matrix(np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])).as_quat()
        # self.aruco2base0.rotation.x = quat0[0]
        # self.aruco2base0.rotation.y = quat0[1]
        # self.aruco2base0.rotation.z = quat0[2]
        # self.aruco2base0.rotation.w = quat0[3]

        self.aruco2base = Transform()
        self.aruco2base.translation.x = 0.10
        self.aruco2base.translation.y = 0.0
        self.aruco2base.translation.z = 0.0
        quat = R.from_matrix(np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])).as_quat()
        self.aruco2base.rotation.x = quat[0]
        self.aruco2base.rotation.y = quat[1]
        self.aruco2base.rotation.z = quat[2]
        self.aruco2base.rotation.w = quat[3]

        self.aruco2base0 = Transform()
        self.aruco2base0.translation.x = 0.13
        self.aruco2base0.translation.y = 0.0
        self.aruco2base0.translation.z = 0.035
        quat0 = R.from_matrix(np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])).as_quat()
        self.aruco2base0.rotation.x = quat0[0]
        self.aruco2base0.rotation.y = quat0[1]
        self.aruco2base0.rotation.z = quat0[2]
        self.aruco2base0.rotation.w = quat0[3]

        self.aruco2base1 = Transform()
        self.aruco2base1.translation.x = 0.0
        self.aruco2base1.translation.y = 0.07
        self.aruco2base1.translation.z = 0.035
        quat1 = R.from_matrix(np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])).as_quat()
        self.aruco2base1.rotation.x = quat1[0]
        self.aruco2base1.rotation.y = quat1[1]
        self.aruco2base1.rotation.z = quat1[2]
        self.aruco2base1.rotation.w = quat1[3]

        self.aruco2base2 = Transform()
        self.aruco2base2.translation.x = -0.13
        self.aruco2base2.translation.y = 0.0
        self.aruco2base2.translation.z = 0.035
        quat2 = R.from_matrix(np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])).as_quat()
        self.aruco2base2.rotation.x = quat2[0]
        self.aruco2base2.rotation.y = quat2[1]
        self.aruco2base2.rotation.z = quat2[2]
        self.aruco2base2.rotation.w = quat2[3]

        self.aruco2base3 = Transform()
        self.aruco2base3.translation.x = 0.0
        self.aruco2base3.translation.y = -0.07
        self.aruco2base3.translation.z = 0.035
        quat3 = R.from_matrix(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])).as_quat()
        self.aruco2base3.rotation.x = quat3[0]
        self.aruco2base3.rotation.y = quat3[1]
        self.aruco2base3.rotation.z = quat3[2]
        self.aruco2base3.rotation.w = quat3[3]

        # List of bot IDs, if bot, report transform. Otherwise, just report the aruco ID.
        self.bots = [94,89,84,85,91,90,88,82,81,87,83,92,93,86]

        # Report.
        self.get_logger().info("ArUco detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def info_cb(self, msg):
        if not self.caminfoready:
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(msg.k).reshape((3,3))
            self.caminfoready = True
            self.get_logger().info("Camera info received and loaded in!")


    def group_aruco_ids(self, aruco_ids):
        grouped_ids = {}
        for id in aruco_ids:
            if id//4 not in grouped_ids:
                grouped_ids[id//4] = [id]
            else:
                grouped_ids[id//4] += [id]
        return grouped_ids


    # Process the image (detect the aruco).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Detect.
        (boxes, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.dict, parameters=self.params)

        # Loop over each marker: the list of corners and ID for this marker.
        if len(boxes) > 0:
            boxes_and_ids = zip(boxes, ids.flatten())
            for (box, id) in boxes_and_ids:

                # The corners are top-left, top-right, bottom-right,
                # bottom-left of the original marker (not the bounding
                # box).  Each is a 2x1 numpy array of pixel
                # coordinates.  Their type is floating-point though
                # they contain integer values.
                (topLeft, topRight, bottomRight, bottomLeft) = box[0]
                center = (topLeft + bottomRight)/2

                # Draw the box around the marker.
                pts = [box.reshape(-1,1,2).astype(int)]
                cv2.polylines(frame, pts, True, self.green, 2)
            
                # Draw the center circle and write the ID there.
                ctr = tuple(center.astype(int))
                cv2.circle(frame, ctr, 4, self.red, -1)
                cv2.putText(frame, str(id), ctr,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 2)
                
        if len(boxes) > 0 and self.caminfoready:

            # Average over multiple aruco markers if you see a few
            if len(ids.flatten()) > 0:
                seen_dots = set()
                for bot_id, box in zip(ids.flatten(), boxes):
                    rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(box, 0.04, np.array(self.camK).reshape((3, 3)), np.array(self.camD))
                    cv2.drawFrameAxes(frame, np.array(self.camK).reshape((3, 3)), np.array(self.camD), rvecs, tvecs, 0.04)

                    count = 0
                    tvec_avg = np.array([0.0, 0.0, 0.0])

                    if int(bot_id) in self.bots:
                        for i in range(0, tvecs.shape[1]):
                            count += 1
                            tvec = tvecs[0, i, :] # NOTE IF ERROR HERE, MAY BE BECAUSE MULTIPLE TAGS RETURNED??
                            rvec = rvecs[i, :, :]

                            aruco2camera = Transform()
                            aruco2camera.translation.x = tvec[0]
                            aruco2camera.translation.y = tvec[1]
                            aruco2camera.translation.z = tvec[2]
                            rmat = cv2.Rodrigues(rvec)[0]
                            quat_aruco2camera = R.from_matrix(rmat).as_quat()
                            aruco2camera.rotation.x = quat_aruco2camera[0]
                            aruco2camera.rotation.y = quat_aruco2camera[1]
                            aruco2camera.rotation.z = quat_aruco2camera[2]
                            aruco2camera.rotation.w = quat_aruco2camera[3]

                            botbase2aruco = tf_inv(self.aruco2base)

                            botbase2camera = tf_mult(aruco2camera, botbase2aruco)

                            tvec_avg += np.array([aruco2camera.translation.x, aruco2camera.translation.y, aruco2camera.translation.z])

                        tvec_avg = tvec_avg / count

                        aruco_pose_msg = PointWithIDStamped()
                        aruco_pose_msg.id = int(bot_id)
                        aruco_pose_msg.header = Header()
                        aruco_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        aruco_pose_msg.header.frame_id = 'camera'
                    
                        aruco_pose_msg.point.x = tvec_avg[2]
                        aruco_pose_msg.point.y = tvec_avg[0]
                        aruco_pose_msg.point.z = tvec_avg[1]

                        self.get_logger().info("Detecting bot with ID: "+str(bot_id))
                        self.pub_obs.publish(aruco_pose_msg)

                    elif int(bot_id) not in seen_dots:
                        seen_dots.add(int(bot_id))
                        aruco_pose_msg = PointWithIDStamped()
                        aruco_pose_msg.id = int(bot_id)
                        aruco_pose_msg.header = Header()
                        aruco_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        aruco_pose_msg.header.frame_id = 'camera'

                        self.get_logger().info("Detecting pacdot ID: "+str(bot_id))
                        self.get_logger().info("seen_dot: "+str(seen_dots))

                        self.pub_dots.publish(aruco_pose_msg)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('detector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
