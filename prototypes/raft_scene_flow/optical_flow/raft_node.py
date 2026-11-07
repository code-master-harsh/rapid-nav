#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import torch
import torchvision

from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge


class RAFTNode(Node):

    def __init__(self):
        super().__init__('raft_node')

        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # Latest RGB / point cloud
        # ---------------------------------------------------------

        self.latest_rgb = None
        self.latest_cloud = None

        # Previous processed frame
        self.prev_rgb = None
        self.prev_xyz = None
        self.prev_stamp = None

        # Prevent multiple callbacks from processing the same pair
        self.processing = False

        # ---------------------------------------------------------
        # RAFT
        # ---------------------------------------------------------

        self.device = torch.device('cpu')

        self.get_logger().info('Loading RAFT Small on CPU...')

        weights = torchvision.models.optical_flow.Raft_Small_Weights.DEFAULT

        self.raft = torchvision.models.optical_flow.raft_small(
            weights=weights
        )

        self.raft = self.raft.to(self.device)
        self.raft.eval()

        self.transforms = weights.transforms()

        self.get_logger().info('RAFT Small loaded on CPU')

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgbd/image',
            self.image_callback,
            10
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/rgbd/points',
            self.pointcloud_callback,
            10
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.flow_pub = self.create_publisher(
            Image,
            '/optical_flow',
            10
        )

        self.points_pub = self.create_publisher(
            PointCloud2,
            '/raft/points',
            10
        )

        self.velocity_pub = self.create_publisher(
            PointCloud2,
            '/raft/velocity_field',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/raft_flow/markers',
            10
        )

        self.get_logger().info('RAFT node ready')

    # =============================================================
    # RGB CALLBACK
    # =============================================================

    def image_callback(self, msg):

        self.latest_rgb = msg

        self.try_process()

    # =============================================================
    # POINT CLOUD CALLBACK
    # =============================================================

    def pointcloud_callback(self, msg):

        self.latest_cloud = msg

        self.try_process()

    # =============================================================
    # PROCESS LATEST RGB + POINT CLOUD
    # =============================================================

    def try_process(self):

        if self.processing:
            return

        if self.latest_rgb is None:
            return

        if self.latest_cloud is None:
            return

        self.processing = True

        rgb_msg = self.latest_rgb
        cloud_msg = self.latest_cloud

        # Clear these so the same messages aren't processed repeatedly
        self.latest_rgb = None
        self.latest_cloud = None

        try:
            self.process_pair(rgb_msg, cloud_msg)

        except Exception as e:

            self.get_logger().error(
                f'Processing error: {e}'
            )

        finally:
            self.processing = False

    # =============================================================
    # PROCESS FRAME PAIR
    # =============================================================

    def process_pair(self, rgb_msg, cloud_msg):

        rgb = self.image_to_numpy(rgb_msg)

        xyz = self.pointcloud_to_xyz(cloud_msg)

        height, width, _ = xyz.shape

        # ---------------------------------------------------------
        # Timestamp
        # ---------------------------------------------------------

        stamp = (
            cloud_msg.header.stamp.sec
            + cloud_msg.header.stamp.nanosec * 1e-9
        )

        # ---------------------------------------------------------
        # First frame
        # ---------------------------------------------------------

        if self.prev_rgb is None:

            self.prev_rgb = rgb.copy()
            self.prev_xyz = xyz.copy()
            self.prev_stamp = stamp

            self.get_logger().info(
                f'First frame received: {width}x{height}'
            )

            return

        # ---------------------------------------------------------
        # Time difference
        # ---------------------------------------------------------

        dt = stamp - self.prev_stamp

        if dt <= 0:

            self.get_logger().warn(
                'Invalid timestamp difference'
            )

            return

        # ---------------------------------------------------------
        # Make sure RGB and point cloud dimensions match
        # ---------------------------------------------------------

        if (
            self.prev_xyz.shape[0] != self.prev_rgb.shape[0]
            or
            self.prev_xyz.shape[1] != self.prev_rgb.shape[1]
        ):

            self.get_logger().warn(
                'Previous RGB / point cloud dimensions do not match'
            )

            self.prev_rgb = rgb.copy()
            self.prev_xyz = xyz.copy()
            self.prev_stamp = stamp

            return

        if (
            xyz.shape[0] != rgb.shape[0]
            or
            xyz.shape[1] != rgb.shape[1]
        ):

            self.get_logger().warn(
                'Current RGB / point cloud dimensions do not match'
            )

            self.prev_rgb = rgb.copy()
            self.prev_xyz = xyz.copy()
            self.prev_stamp = stamp

            return

        # =========================================================
        # RAFT FLOW
        # =========================================================

        flow = self.compute_raft_flow(
            self.prev_rgb,
            rgb
        )

        # flow shape:
        #
        # H x W x 2
        #
        # flow[v,u,0] = du
        # flow[v,u,1] = dv

        self.publish_flow(
            flow,
            rgb_msg.header
        )

        # =========================================================
        # SOURCE POINT CLOUD
        # =========================================================

        source_points = []

        # Keep a subset for performance
        step = 4

        for v in range(0, height, step):

            for u in range(0, width, step):

                x, y, z = self.prev_xyz[v, u]

                if not np.isfinite(x):
                    continue

                if not np.isfinite(y):
                    continue

                if not np.isfinite(z):
                    continue

                if z == 0:
                    continue

                source_points.append(
                    (
                        float(x),
                        float(y),
                        float(z),
                        float(u),
                        float(v)
                    )
                )

        # Publish colored source point cloud
        self.publish_source_cloud(
            source_points,
            cloud_msg.header,
            self.prev_rgb
        )

        # =========================================================
        # VELOCITY FIELD
        # =========================================================

        velocity_points = []

        marker_array = MarkerArray()

        marker = Marker()

        marker.header = cloud_msg.header
        marker.ns = 'raft_velocity'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.005

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # ---------------------------------------------------------
        # For each source point:
        #
        # P1 = previous point cloud
        #
        # flow -> destination pixel
        #
        # P2 = current point cloud at destination pixel
        #
        # V = (P2 - P1) / dt
        # ---------------------------------------------------------

        for v in range(0, height, step):

            for u in range(0, width, step):

                p1 = self.prev_xyz[v, u]

                if not np.all(np.isfinite(p1)):
                    continue

                du = flow[v, u, 0]
                dv = flow[v, u, 1]

                if not np.isfinite(du) or not np.isfinite(dv):
                    continue

                u2 = int(round(u + du))
                v2 = int(round(v + dv))

                # Destination outside image
                if u2 < 0 or u2 >= width:
                    continue

                if v2 < 0 or v2 >= height:
                    continue

                p2 = xyz[v2, u2]

                if not np.all(np.isfinite(p2)):
                    continue

                # Ignore invalid zero depth
                if np.linalg.norm(p1) < 1e-6:
                    continue

                if np.linalg.norm(p2) < 1e-6:
                    continue

                # -------------------------------------------------
                # Velocity
                # -------------------------------------------------

                velocity = (p2 - p1) / dt

                if not np.all(np.isfinite(velocity)):
                    continue

                vx = float(velocity[0])
                vy = float(velocity[1])
                vz = float(velocity[2])

                # Prevent completely insane values from
                # dominating visualization
                velocity_norm = np.linalg.norm(velocity)

                if velocity_norm > 100.0:
                    continue

                # -------------------------------------------------
                # [x,y,z,vx,vy,vz]
                # -------------------------------------------------

                velocity_points.append(
                    (
                        float(p1[0]),
                        float(p1[1]),
                        float(p1[2]),
                        vx,
                        vy,
                        vz
                    )
                )

                # -------------------------------------------------
                # Marker: P1 -> P2
                # -------------------------------------------------

                marker.points.append(
                    self.make_point(
                        float(p1[0]),
                        float(p1[1]),
                        float(p1[2])
                    )
                )

                marker.points.append(
                    self.make_point(
                        float(p2[0]),
                        float(p2[1]),
                        float(p2[2])
                    )
                )

        # =========================================================
        # PUBLISH VELOCITY FIELD
        # =========================================================

        self.publish_velocity_cloud(
            velocity_points,
            cloud_msg.header
        )

        marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # =========================================================
        # UPDATE PREVIOUS FRAME
        # =========================================================

        self.prev_rgb = rgb.copy()
        self.prev_xyz = xyz.copy()
        self.prev_stamp = stamp

        self.get_logger().info(
            f'Processed frame: '
            f'flow={flow.shape}, '
            f'velocity vectors={len(velocity_points)}, '
            f'dt={dt:.4f}s'
        )

    # =============================================================
    # IMAGE -> NUMPY
    # =============================================================

    def image_to_numpy(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='rgb8'
        )

        return np.asarray(
            image,
            dtype=np.uint8
        )

    # =============================================================
    # POINTCLOUD2 -> XYZ
    # =============================================================

    def pointcloud_to_xyz(self, msg):

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )

        xyz = np.stack(
            [
                points['x'],
                points['y'],
                points['z']
            ],
            axis=-1
        ).astype(np.float32)

        xyz = xyz.reshape(
            msg.height,
            msg.width,
            3
        )

        return xyz

    # =============================================================
    # RAFT
    # =============================================================

    def compute_raft_flow(
        self,
        previous_rgb,
        current_rgb
    ):

        # Convert HWC uint8 -> CHW float
        img1 = torch.from_numpy(
            previous_rgb
        ).permute(
            2, 0, 1
        )

        img2 = torch.from_numpy(
            current_rgb
        ).permute(
            2, 0, 1
        )

        img1 = img1.float() / 255.0
        img2 = img2.float() / 255.0

        # RAFT expects batched tensors
        img1 = img1.unsqueeze(0)
        img2 = img2.unsqueeze(0)

        # Apply pretrained RAFT transforms
        img1, img2 = self.transforms(
            img1,
            img2
        )

        img1 = img1.to(self.device)
        img2 = img2.to(self.device)

        with torch.no_grad():

            flow_predictions = self.raft(
                img1,
                img2
            )

        # Use final prediction
        flow = flow_predictions[-1]

        # B x 2 x H x W -> H x W x 2
        flow = flow[0].permute(
            1,
            2,
            0
        )

        flow = flow.cpu().numpy()

        return flow.astype(np.float32)

    # =============================================================
    # PUBLISH OPTICAL FLOW
    # =============================================================

    def publish_flow(
        self,
        flow,
        header
    ):

        msg = Image()

        msg.header = header
        msg.height = flow.shape[0]
        msg.width = flow.shape[1]
        msg.encoding = '32FC2'
        msg.is_bigendian = False
        msg.step = flow.shape[1] * 2 * 4

        msg.data = flow.astype(
            np.float32
        ).tobytes()

        self.flow_pub.publish(msg)

    # =============================================================
    # PUBLISH COLORED SOURCE POINT CLOUD
    # =============================================================

    def publish_source_cloud(
        self,
        points,
        header,
        rgb_image
    ):

        if len(points) == 0:
            return

        fields = [

            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='rgb',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        cloud_data = []

        for x, y, z, u, v in points:

            u = int(u)
            v = int(v)

            if (
                v < 0
                or v >= rgb_image.shape[0]
                or u < 0
                or u >= rgb_image.shape[1]
            ):
                continue

            r, g, b = rgb_image[v, u]

            rgb_uint = (
                (int(r) << 16)
                |
                (int(g) << 8)
                |
                int(b)
            )

            rgb_float = np.frombuffer(
                np.uint32(rgb_uint).tobytes(),
                dtype=np.float32
            )[0]

            cloud_data.append(
                (
                    float(x),
                    float(y),
                    float(z),
                    rgb_float
                )
            )

        if len(cloud_data) == 0:
            return

        msg = point_cloud2.create_cloud(
            header,
            fields,
            cloud_data
        )

        self.points_pub.publish(msg)

    # =============================================================
    # PUBLISH VELOCITY POINT CLOUD
    # =============================================================

    def publish_velocity_cloud(
        self,
        points,
        header
    ):

        if len(points) == 0:
            return

        fields = [

            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='vx',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='vy',
                offset=16,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='vz',
                offset=20,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        msg = point_cloud2.create_cloud(
            header,
            fields,
            points
        )

        self.velocity_pub.publish(msg)

    # =============================================================
    # MARKER POINT
    # =============================================================

    def make_point(self, x, y, z):

        from geometry_msgs.msg import Point

        p = Point()

        p.x = x
        p.y = y
        p.z = z

        return p


def main(args=None):

    rclpy.init(args=args)

    node = RAFTNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()