import rclpy
import csv
from scipy import stats
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import matplotlib.pyplot as plt
import subprocess
import math

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

class BatteryNotifierNode(Node):
    def __init__(self):
        super().__init__('battery_notifier_node')
        
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data
        
        #subscribe to battery warning topic
        self.warning_subscriber = self.create_subscription(Bool, '/battery_warning', self.warning_callback, SENSOR_QOS)

    def warning_callback(self, msg: Bool):
        # flashes LEDs and other visual indicators to alert the user of low battery
        if msg.data:
            pass
        pass