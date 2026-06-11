#!/usr/bin/env python3

'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2026
#
# author: Giorgio Marmolino
# mail: giorgio.marmolino@gmail.com
#
# This file is part of camera_utils_ros. <https://github.com/IASRobolab/camera_utils_ros>
# GPLv3 - see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''

from camera_utils.cameras.IntelRealsense import IntelRealsense
from camera_utils.cameras.Zed import Zed
from camera_utils.cameras.Webcam import Webcam
from camera_utils.cameras.CameraInterface import Camera

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from camera_utils_msgs.msg import Frames
from cv_bridge import CvBridge


class CameraPublisher(Node):

    def __init__(self):
        super().__init__("camera_publisher")

        self.bridge = CvBridge()

        Resolution = Camera.Resolution

        # --- parameters ---
        self.declare_parameter("rgb_topic", "rgb_image_raw")
        self.declare_parameter("depth_topic", "depth_image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("camera_pcd_topic", "camera_pcd")
        self.declare_parameter("frames_topic", "camera_frames")
        self.declare_parameter("camera_resolution", "HD")
        self.declare_parameter("compressed_image", False)
        self.declare_parameter("fps", 30)
        self.declare_parameter("serial_number", "")
        self.declare_parameter("publish_rgb", False)
        self.declare_parameter("publish_depth", False)
        self.declare_parameter("publish_camera_info", False)
        self.declare_parameter("publish_separated_frames", True)
        self.declare_parameter("camera_type", "")
        self.declare_parameter("device_idx", 0)
        # --- QoS configurabile ---
        self.declare_parameter("reliability", "best_effort")  # "best_effort" | "reliable"
        self.declare_parameter("qos_depth", 5)

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        frames_topic = self.get_parameter("frames_topic").value

        camera_resolution = self.get_parameter("camera_resolution").value
        camera_resolution = eval("Resolution." + camera_resolution)
        self.compressed_image = self.get_parameter("compressed_image").value

        fps = self.get_parameter("fps").value
        serial_number = self.get_parameter("serial_number").value

        self.publish_rgb = self.get_parameter("publish_rgb").value
        self.publish_depth = self.get_parameter("publish_depth").value
        self.publish_camera_info = self.get_parameter("publish_camera_info").value
        self.publish_separated_frames = self.get_parameter("publish_separated_frames").value

        camera_type = self.get_parameter("camera_type").value
        device_idx = self.get_parameter("device_idx").value

        # --- build del QoSProfile per i topic immagine/frames ---
        reliability = str(self.get_parameter("reliability").value).lower()
        qos_depth = self.get_parameter("qos_depth").value

        if reliability == "reliable":
            rel_policy = QoSReliabilityPolicy.RELIABLE
        elif reliability == "best_effort":
            rel_policy = QoSReliabilityPolicy.BEST_EFFORT
        else:
            self.get_logger().error(
                "reliability '%s' non valida: usa 'best_effort' o 'reliable'" % reliability)
            raise SystemExit

        image_qos = QoSProfile(
            reliability=rel_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )
        self.get_logger().info(
            "QoS immagini: reliability=%s, depth=%d" % (reliability, qos_depth))

        # CameraInfo: convenzione = latched (transient_local + reliable), cosi' i
        # subscriber tardivi ricevono gli intrinsics. Indipendente dalla scelta sopra.
        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        if not camera_type:
            self.get_logger().error("No camera type passed")
            raise SystemExit

        self.depth_encoding = None

        if camera_type == "intel":
            self.camera = IntelRealsense(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
            self.depth_encoding = "mono16"
        elif camera_type == "zed":
            self.camera = Zed(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
            self.depth_encoding = "32FC1"
        elif camera_type == "webcam":
            self.camera = Webcam(device_idx)
        else:
            self.get_logger().error("Camera Type " + camera_type + " does not exists!")
            raise SystemExit

        self.rgb_publisher = None
        self.depth_publisher = None
        self.camera_info_publisher = None
        self.frames_publisher = None

        image_type = Image

        if self.compressed_image:
            image_type = CompressedImage
            rgb_topic += "/compressed"

        if self.publish_depth and self.publish_rgb:
            if self.publish_separated_frames:
                self.depth_publisher = self.create_publisher(image_type, depth_topic, image_qos)
                self.rgb_publisher = self.create_publisher(image_type, rgb_topic, image_qos)
            else:
                self.frames_publisher = self.create_publisher(Frames, frames_topic, image_qos)
        elif self.publish_depth:
            self.depth_publisher = self.create_publisher(image_type, depth_topic, image_qos)
        elif self.publish_rgb:
            self.rgb_publisher = self.create_publisher(image_type, rgb_topic, image_qos)

        if self.publish_camera_info and self.publish_separated_frames:
            self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic, camera_info_qos)

        # --- intrinsics ---
        self.camera_info = CameraInfo()
        try:
            intr = self.camera.get_intrinsics()
            # in ROS 2 il campo della matrice intrinseca e' 'k' (minuscolo), non 'K'
            self.camera_info.k[0] = intr['fx']
            self.camera_info.k[4] = intr['fy']
            self.camera_info.k[2] = intr['px']
            self.camera_info.k[5] = intr['py']
            self.camera_info.width = intr['width']
            self.camera_info.height = intr['height']
        except KeyError:
            pass

        self.rgb_cv2_to_imgmsg = self.bridge.cv2_to_imgmsg
        self.rgb_encoding = "bgr8"
        if self.compressed_image:
            self.rgb_cv2_to_imgmsg = self.bridge.cv2_to_compressed_imgmsg
            self.rgb_encoding = "jpg"

    def run(self):
        while rclpy.ok():

            t = self.get_clock().now().to_msg()

            if self.publish_depth and self.publish_rgb:
                rgb, depth = self.camera.get_frames()

                rgb_image = self.rgb_cv2_to_imgmsg(rgb, self.rgb_encoding)
                depth_image = self.bridge.cv2_to_imgmsg(depth, self.depth_encoding)

                rgb_image.header.stamp = t
                depth_image.header.stamp = t

                if self.publish_separated_frames:
                    self.rgb_publisher.publish(rgb_image)
                    self.depth_publisher.publish(depth_image)
                else:
                    frames = Frames()
                    frames.header.stamp = t
                    frames.rgb = rgb_image
                    frames.depth = depth_image
                    frames.camera_info = self.camera_info
                    self.frames_publisher.publish(frames)

            elif self.publish_rgb:
                rgb = self.camera.get_rgb()

                rgb_image = self.rgb_cv2_to_imgmsg(rgb, self.rgb_encoding)
                rgb_image.header.stamp = t
                self.rgb_publisher.publish(rgb_image)

            elif self.publish_depth:
                depth = self.camera.get_depth()

                depth_image = self.bridge.cv2_to_imgmsg(depth, self.depth_encoding)
                depth_image.header.stamp = t
                self.depth_publisher.publish(depth_image)

            if self.publish_camera_info and self.publish_separated_frames:
                self.camera_info_publisher.publish(self.camera_info)

            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        node.run()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()