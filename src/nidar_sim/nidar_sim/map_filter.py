import numpy as np

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)

from nav_msgs.msg import OccupancyGrid

LATCHED = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

class MapFilter(Node):
    def __init__(self):
        super().__init__('map_filter')

        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_filtered')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_blob_cells', 12)
        self.declare_parameter('report_period_sec', 20.0)

        g = lambda n: self.get_parameter(n).value
        self.occ_thresh = int(g('occupied_threshold'))
        self.min_blob = int(g('min_blob_cells'))
        out_topic = g('output_topic')

        self.pub = self.create_publisher(OccupancyGrid, out_topic, LATCHED)
        self.sub = self.create_subscription(
            OccupancyGrid, g('input_topic'), self._cb, LATCHED)

        self._maps = 0
        self._removed_blobs = 0
        self._removed_cells = 0
        self._kept_blobs = 0
        self._largest_removed = 0
        self.create_timer(float(g('report_period_sec')), self._report)

        self.get_logger().info(
            f"map_filter up: {g('input_topic')} -> {out_topic}. Occupied "
            f"components smaller than {self.min_blob} cells "
            f"({self.min_blob * 0.0025:.3f} m2 at 0.05 m) are speckle and are "
            f"cleared; anything touching a larger structure is kept.")

    def _cb(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)

        occ = (data >= self.occ_thresh).astype(np.uint8)
        if occ.any():
            n, labels, stats, _ = cv2.connectedComponentsWithStats(
                occ, connectivity=8)
            sizes = stats[1:, cv2.CC_STAT_AREA]
            small = np.nonzero(sizes < self.min_blob)[0] + 1
            if small.size:
                kill = np.isin(labels, small)
                data = data.copy()
                data[kill] = 0
                self._removed_blobs += int(small.size)
                self._removed_cells += int(kill.sum())
                self._largest_removed = max(
                    self._largest_removed, int(sizes[small - 1].max()))
            self._kept_blobs += int(n - 1 - small.size)
        self._maps += 1

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = data.reshape(-1).astype(np.int8).tolist()
        self.pub.publish(out)

    def _report(self):
        if not self._maps:
            self.get_logger().warn(
                'No maps received yet. Is slam_toolbox publishing, and does '
                'its QoS match (TRANSIENT_LOCAL)?')
            return
        self.get_logger().info(
            f'{self._maps} maps | removed {self._removed_blobs} speckle '
            f'island(s), {self._removed_cells} cells, largest '
            f'{self._largest_removed} cells | kept {self._kept_blobs} '
            f'structure(s)')
        self._maps = 0
        self._removed_blobs = 0
        self._removed_cells = 0
        self._kept_blobs = 0
        self._largest_removed = 0

def main(args=None):
    rclpy.init(args=args)
    node = MapFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
