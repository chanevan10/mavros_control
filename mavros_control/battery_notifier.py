import rclpy
import csv
from scipy import stats
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import matplotlib.pyplot as plt
import subprocess
import math
import time
import threading
from typing import Sequence, Tuple

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32


class BatteryNotifierNode(Node):
    def __init__(self):
        super().__init__('battery_notifier_node')

        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # subscribe to battery warning topic
        self.warning_subscriber = self.create_subscription(Bool, '/battery_warning', self.warning_callback, SENSOR_QOS)

        # publish light levels to the headlights
        self.pub = self.create_publisher(Float32, 'desired_light_level', 10)

        # flashing state thread management
        self._flashing = False
        self._flash_lock = threading.Lock()

    def _publish_level(self, level: float):
        msg = Float32()
        msg.data = float(level)
        self.pub.publish(msg)

    def _flash_once(self, on_dur: float, off_dur: float):
        self._publish_level(1.0)
        time.sleep(on_dur)
        self._publish_level(0.0)
        time.sleep(off_dur)

    def _flash_repeats(self, repeats: int, on_dur: float, off_dur: float):
        for _ in range(repeats):
            self._flash_once(on_dur, off_dur)

    def _flash_sequence(self, sequence: Sequence[Tuple[int, float, float]], gap_between_groups: float = 0.2):
        for repeats, on_dur, off_dur in sequence:
            self._flash_repeats(repeats, on_dur, off_dur)
            time.sleep(gap_between_groups)

    def start_flashing_thread(self, sequence: Sequence[Tuple[int, float, float]], gap_between_groups: float = 0.2):
        # avoid overlapping flash sequences
        with self._flash_lock:
            if self._flashing:
                return
            self._flashing = True

        def _worker():
            try:
                self._flash_sequence(sequence, gap_between_groups)
            finally:
                with self._flash_lock:
                    self._flashing = False

        thr = threading.Thread(target=_worker, daemon=True)
        thr.start()

    def warning_callback(self, msg: Bool):
        # flashes headlights and other visual indicators to alert the user of low battery
        if msg.data:
            # Example pattern: 3x0.1s, then 3x0.2s, then 3x0.1s
            sequence = [(3, 0.1, 0.1), (3, 0.2, 0.2), (3, 0.1, 0.1)]
            self.start_flashing_thread(sequence, gap_between_groups=0.25)
    
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNotifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()