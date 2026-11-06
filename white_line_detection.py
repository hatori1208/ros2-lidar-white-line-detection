"""地面点群の反射強度とDBSCANを用いた白線認識（ROS 2）。

入力:
    /pcd_segment_ground (sensor_msgs/PointCloud2)
    /odom/wheel_imu (nav_msgs/Odometry、パラメータ ``odom`` で変更可能)
出力:
    /white_lines, /white_line_solid, /white_line_dashed (sensor_msgs/PointCloud2)

元のスクリプトから反射強度マップの作成・保存と未使用コードを除去した版。
白線判定の閾値・蓄積時間・分類条件は元の実装と同じ。
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header


class ReflectionIntensityMap(Node):
    """反射強度で点を抽出し、DBSCANで実線・破線を分類する。"""

    def __init__(self):
        # 既存の起動設定と互換性を保つため、ノード名とトピック名は変更しない。
        super().__init__('reflection_intensity_map_node')

        cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.declare_parameter('odom', '/odom/wheel_imu')
        odom_topic = self.get_parameter('odom').get_parameter_value().string_value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.get_odom, odom_qos)
        self.subscription = self.create_subscription(
            PointCloud2, '/pcd_segment_ground', self.reflect_map, cloud_qos
        )

        self.white_line = self.create_publisher(PointCloud2, 'white_lines', 10)
        self.white_solid_pub = self.create_publisher(PointCloud2, 'white_line_solid', 10)
        self.white_dashed_pub = self.create_publisher(PointCloud2, 'white_line_dashed', 10)

        # global（odom座標系）で保持。行は x, y, z, intensity, timestamp。
        self.pcd_ground_buff = np.empty((5, 0), dtype=np.float64)
        self.duration = 3.0
        self.white_result_buff = np.empty((5, 0), dtype=np.float64)
        self.white_result_duration = 2.0

        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.theta_z = 0.0  # deg（元の実装と同じくroll/pitchは反映しない）

        self.ground_pixel = 1000 / 50  # 5cm刻みの座標丸め
        self.MAP_LIM_X_MIN = -30.0
        self.MAP_LIM_X_MAX = 30.0
        self.MAP_LIM_Y_MIN = -30.0
        self.MAP_LIM_Y_MAX = 30.0

        self.intensity_threshold = 40.0
        self.dbscan_eps = 0.30
        self.dbscan_min_samples = 6
        self.cluster_size_threshold = 25

        self.get_logger().info('reflection_to_pcd_dbscan (white-line-only) started')

    def get_odom(self, msg):
        """点群をodom座標系に変換するための位置とyawを更新する。"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta_z = math.degrees(yaw)

    @staticmethod
    def pointcloud2_to_array(cloud_msg):
        """x, y, z, intensity が先頭からfloat32で並ぶPointCloud2を読み取る。"""
        points = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(
            -1, cloud_msg.point_step
        )
        x = np.frombuffer(points[:, 0:4].tobytes(), dtype=np.float32)
        y = np.frombuffer(points[:, 4:8].tobytes(), dtype=np.float32)
        z = np.frombuffer(points[:, 8:12].tobytes(), dtype=np.float32)
        intensity = np.frombuffer(points[:, 12:16].tobytes(), dtype=np.float32)
        return np.vstack((x, y, z, intensity))

    def reflect_map(self, msg):
        """点群の座標変換、時間蓄積、DBSCAN処理を実行する。"""
        t_stamp = msg.header.stamp
        points = self.pointcloud2_to_array(msg)
        position_x, position_y = self.position_x, self.position_y

        # 元の実装と同様、roll/pitch=0としてyawだけで回転する。
        ground_rot, _ = rotation_xyz(points[:3, :], 0.0, 0.0, self.theta_z)
        ground_global = np.vstack((
            ground_rot[0, :] + position_x,
            ground_rot[1, :] + position_y,
            ground_rot[2, :],
            points[3, :],
        )).astype(np.float32)

        current_time = t_stamp.sec + t_stamp.nanosec * 1e-9

        # 古い点を削除。
        if self.pcd_ground_buff.shape[1] > 0:
            time_valid = self.pcd_ground_buff[4, :] > (current_time - self.duration)
            self.pcd_ground_buff = self.pcd_ground_buff[:, time_valid]

        # 現在位置を中心とした±30mの範囲内に蓄積済みの点を限定。
        if self.pcd_ground_buff.shape[1] > 0:
            area_valid = self.pcd_serch(
                self.pcd_ground_buff,
                position_x + self.MAP_LIM_X_MIN,
                position_x + self.MAP_LIM_X_MAX,
                position_y + self.MAP_LIM_Y_MIN,
                position_y + self.MAP_LIM_Y_MAX,
            )
            self.pcd_ground_buff = self.pcd_ground_buff[:, area_valid]

        time_array = np.full((1, ground_global.shape[1]), current_time, dtype=np.float64)
        ground_global_with_time = np.vstack((ground_global, time_array))
        if self.pcd_ground_buff.shape[1] == 0:
            self.pcd_ground_buff = ground_global_with_time
        else:
            self.pcd_ground_buff = np.hstack((self.pcd_ground_buff, ground_global_with_time))

        # 蓄積点群の座標を5cm単位に丸める（intensity/timeは変更しない）。
        self.pcd_ground_buff[:3, :] = (
            np.round(self.pcd_ground_buff[:3, :] * self.ground_pixel)
            / self.ground_pixel
        )
        self.process_dbscan(t_stamp)

    def process_dbscan(self, t_stamp):
        """反射強度で候補を絞り、DBSCANのクラスタを実線と破線に分類する。"""
        try:
            if self.pcd_ground_buff.shape[1] == 0:
                return

            white_ind = self.pcd_ground_buff[3, :] > self.intensity_threshold
            white_points = self.pcd_ground_buff[:4, :][:, white_ind]
            if white_points.shape[1] == 0:
                return

            labels = DBSCAN(
                eps=self.dbscan_eps,
                min_samples=self.dbscan_min_samples,
            ).fit_predict(white_points[:2, :].T)

            valid_mask = labels != -1
            clustered_points = white_points[:, valid_mask]
            clustered_labels = labels[valid_mask]
            if clustered_points.shape[1] == 0:
                return

            solid_clusters = []
            dashed_clusters = []
            for label in np.unique(clustered_labels):
                cluster = clustered_points[:, clustered_labels == label]
                cluster_size = cluster.shape[1]
                if cluster_size < self.cluster_size_threshold:
                    continue

                spread_x = np.std(cluster[0, :])
                spread_y = np.std(cluster[1, :])

                # 判定条件は元のスクリプトから変更していない。
                if cluster_size > 180 and (spread_x > 0.5 or spread_y > 0.5):
                    solid_clusters.append(cluster)
                elif cluster_size > 30:
                    dashed_clusters.append(cluster)

            if solid_clusters:
                solid_points = np.hstack(solid_clusters)
                self.white_solid_pub.publish(
                    point_cloud_intensity_msg(solid_points.T, t_stamp, 'odom')
                )
            if dashed_clusters:
                dashed_points = np.hstack(dashed_clusters)
                self.white_dashed_pub.publish(
                    point_cloud_intensity_msg(dashed_points.T, t_stamp, 'odom')
                )

            # 白線認識結果を2秒保持して、white_linesとしてPublish。
            all_clusters = solid_clusters + dashed_clusters
            current_time = t_stamp.sec + t_stamp.nanosec * 1e-9
            if self.white_result_buff.shape[1] > 0:
                valid = self.white_result_buff[4, :] > (
                    current_time - self.white_result_duration
                )
                self.white_result_buff = self.white_result_buff[:, valid]

            if all_clusters:
                all_points = np.hstack(all_clusters)
                time_array = np.full((1, all_points.shape[1]), current_time)
                all_points_with_time = np.vstack((all_points, time_array))
                if self.white_result_buff.shape[1] == 0:
                    self.white_result_buff = all_points_with_time
                else:
                    self.white_result_buff = np.hstack((
                        self.white_result_buff, all_points_with_time
                    ))

            if self.white_result_buff.shape[1] > 0:
                publish_points = self.white_result_buff[:4, :]
                self.white_line.publish(
                    point_cloud_intensity_msg(publish_points.T, t_stamp, 'odom')
                )
        except Exception as exc:
            self.get_logger().error(f'DBSCAN処理中にエラーが発生しました: {exc}')

    @staticmethod
    def pcd_serch(pointcloud, x_min, x_max, y_min, y_max):
        """XY範囲に入る点を示す真偽値配列を返す。"""
        return (
            (x_min <= pointcloud[0, :]) & (pointcloud[0, :] <= x_max)
            & (y_min <= pointcloud[1, :]) & (pointcloud[1, :] <= y_max)
        )


def rotation_xyz(pointcloud, theta_x, theta_y, theta_z):
    """オイラー角（度）のXYZ回転を点群に適用する。"""
    theta_x, theta_y, theta_z = map(math.radians, (theta_x, theta_y, theta_z))
    rot_x = np.array([
        [1, 0, 0],
        [0, math.cos(theta_x), -math.sin(theta_x)],
        [0, math.sin(theta_x), math.cos(theta_x)],
    ])
    rot_y = np.array([
        [math.cos(theta_y), 0, math.sin(theta_y)],
        [0, 1, 0],
        [-math.sin(theta_y), 0, math.cos(theta_y)],
    ])
    rot_z = np.array([
        [math.cos(theta_z), -math.sin(theta_z), 0],
        [math.sin(theta_z), math.cos(theta_z), 0],
        [0, 0, 1],
    ])
    rotation_matrix = rot_z @ rot_y @ rot_x
    return rotation_matrix @ pointcloud, rotation_matrix


def quaternion_to_euler(x, y, z, w):
    """姿勢クォータニオンからroll, pitch, yaw（ラジアン）を計算する。"""
    rot_matrix = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)],
    ])
    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(
        rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2
    ))
    yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    return roll, pitch, yaw


def point_cloud_intensity_msg(points, t_stamp, parent_frame):
    """N×4の(x, y, z, intensity)からPointCloud2を生成する。"""
    dtype = np.float32
    point_step = np.dtype(dtype).itemsize * 4
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return PointCloud2(
        header=Header(frame_id=parent_frame, stamp=t_stamp),
        height=1,
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=point_step * points.shape[0],
        data=points.astype(dtype).tobytes(),
    )


def main(args=None):
    rclpy.init(args=args)
    node = ReflectionIntensityMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
