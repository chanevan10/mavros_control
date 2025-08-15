import rclpy
from scipy import stats
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        self.volts = 0.0
        self.volt_buffer = []
        self.current_buffer = []
        self.current = 0.0

        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data


        # Declaring threshold parameters with default values
        description = ParameterDescriptor(description="Voltage threshold for battery warning")
        self.declare_parameter('voltage_threshold', 14.8, description)  # Default threshold, can be changed via ROS parameter
        self.get_logger().info(f"Voltage threshold set to {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, SENSOR_QOS)
        self.reading_timer = self.create_timer(0.1, self.reading_callback)
        
        self.delay_timer = self.create_timer(30, self.start_delay_timer)
        self.calc_timer = self.create_timer(10, self.check_battery_status)

    def start_delay_timer(self):
        self.get_logger().info("Starting battery status checks after 30 seconds delay.")
        self.delay_timer.cancel()
    
    def battery_callback(self, msg: BatteryState):
        # get necessary information from battery topic
        self.volts = msg.voltage
        self.current = msg.current

    # Puts voltage and current readings into buffers every 0.1 seconds
    def reading_callback(self):
        if len(self.volt_buffer) and len(self.current_buffer) > 300:
            self.volt_buffer.pop(0)
            self.current_buffer.pop(0)
        self.volt_buffer.append(self.volts)
        self.current_buffer.append(self.current)

    # Checks battery status via linear regression
    def check_battery_status(self):

        # TODO: filter points based on clustering, remove outliers, make sure points aren't too old

        slope, intercept, r, p, std_err = stats.linregress(self.current_buffer, self.volt_buffer)
        self.get_logger().info(f"Calculated 0A Voltage: {intercept}, Calculated Volts/Current Slope: {slope}")
        if intercept < self.get_parameter('voltage_threshold').get_parameter_value().double_value:
            self.get_logger().warn(f"Battery voltage is below threshold: {intercept} < {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")



        

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
