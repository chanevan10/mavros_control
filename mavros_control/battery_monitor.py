import rclpy
import csv
from scipy import stats
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import matplotlib.pyplot as plt
import math

# imports used to create directories to save plots
import os
import shutil

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        self.volts = 0.0
        self.volt_buffer = []
        self.current_buffer = []
        self.intercepts = []
        self.time_buffer = []

        self.starting_time = self.get_clock().now().to_msg().sec
        self.current = 0.0

        self.graph_dir = f"./battery_data/Back Alley Battery data new"
        # Create directory for storing graphs
        if os.path.exists(self.graph_dir):
            shutil.rmtree(self.graph_dir)  # Remove directory and all its contents
        os.makedirs(self.graph_dir, exist_ok=True)

        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data
        
        # creating csv file and writing header
        with open(os.path.join(self.graph_dir, 'battery_log.csv'), mode='w') as log_file:
            log_writer = csv.writer(log_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            log_writer.writerow(['Time (s)', 'Voltage (V)', 'Current (A)', 'Slope (V/A)', 'Intercept (V)', 'Std Error (V)'])

        # Declaring threshold parameters with default values
        description = ParameterDescriptor(description="Voltage threshold for battery warning")
        self.declare_parameter('voltage_threshold', 14.8, description)  # Default threshold, can be changed via ROS parameter
        self.get_logger().info(f"Voltage threshold set to {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, SENSOR_QOS)
        self.reading_timer = self.create_timer(0.1, self.reading_callback)
        
        self.delay_timer = self.create_timer(30, self.start_delay_timer)
        self.calc_timer = self.create_timer(60, self.check_battery_status)

        # for visualization: make a graph for the readings every 30 seconds
        self.graph_timer = self.create_timer(10, self.graph_readings)

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

            # check slope with buffered values, if slope is not within threshold, reject last reading
            try:
                slope = stats.linregress(self.current_buffer, self.volt_buffer)[0]
            except Exception as e:
                # self.get_logger().error(f"Error calculating linear regression for outlier detection: {e}")
                return

            if slope < 0.22 or slope > 0.29:

                temp_current_buffer = self.current_buffer.copy()
                temp_volt_buffer = self.volt_buffer.copy()

                if slope < 0.22:
                    # find either the bottom most right point or top most left point to remove
                    max_current = max(temp_current_buffer)
                    for i in range(len(temp_current_buffer)):
                        if len(temp_current_buffer) > 2 and temp_current_buffer[i] == max_current:
                            temp_current_buffer.pop(i)
                            temp_volt_buffer.pop(i)
                            break
                if slope > 0.295:
                    # find either the top most right point or bottom most left point to remove
                    min_current = min(temp_current_buffer)
                    for i in range(len(temp_current_buffer)):
                        if len(temp_current_buffer) > 2 and temp_current_buffer[i] == min_current:
                            temp_current_buffer.pop(i)
                            temp_volt_buffer.pop(i)
                            break
                
                if len(temp_current_buffer) > 2 and len(self.intercepts) > 2:
                    new_slope = stats.linregress(temp_current_buffer, temp_volt_buffer)[0]
                    new_intercept = stats.linregress(temp_current_buffer, temp_volt_buffer)[1]
                    if abs(new_intercept-self.intercepts[-1]) < 0.075 and new_slope >= 0.22 and new_slope <= 0.29:
                        self.get_logger().info(f"Rejected outlier reading. Previous slope: {slope}, New slope: {new_slope}")
                        self.current_buffer = temp_current_buffer
                        self.volt_buffer = temp_volt_buffer
                
        self.volt_buffer.append(self.volts)
        self.current_buffer.append(self.current)
        

    # Checks battery status via linear regression
    def check_battery_status(self):

        if self.get_clock().now().to_msg().sec - self.starting_time > 35:
            try:
                slope, intercept, r, p, std_err = stats.linregress(self.current_buffer, self.volt_buffer)
            except Exception as e:
                self.get_logger().error(f"Error calculating linear regression: {e}")
                return

            self.get_logger().info(f"Calculated 0A Voltage: {intercept}, Calculated Volts/Current Slope: {slope}, with standard error: {std_err}")

            if intercept < self.get_parameter('voltage_threshold').get_parameter_value().double_value:
                self.get_logger().warn(f"Battery voltage is below threshold: {intercept} < {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")
            self.intercepts.append(intercept)
            self.time_buffer.append(self.get_clock().now().to_msg().sec - self.starting_time)

    
    def graph_readings(self):
        if self.get_clock().now().to_msg().sec - self.starting_time > 30:
                # Save plots to the created directory
                try:
                    slope, intercept, r, p, std_err = stats.linregress(self.current_buffer, self.volt_buffer)

                     # logging data to csv, not needed for final implementation
                    with open(os.path.join(self.graph_dir, 'battery_log.csv'), mode='a') as log_file:
                        log_writer = csv.writer(log_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        log_writer.writerow([self.get_clock().now().to_msg().sec - self.starting_time, self.volts, self.current, slope, intercept, std_err])

                    # plotting for visualization, not needed for final implementation
                    time_stamp = self.get_clock().now().to_msg().sec - self.starting_time
                    plt.figure()
                    plt.scatter(self.current_buffer, self.volt_buffer, color='blue', label='Data Points')
                    plt.plot(self.current_buffer, [slope * x + intercept for x in self.current_buffer], color='red', label='Regression Line')
                    plt.xlabel('Current (A)')
                    plt.ylabel('Voltage (V)')
                    plt.title('Battery Voltage vs Current with Regression Line')
                    plt.legend()
                    plt.tight_layout()
                    plt.savefig(os.path.join(self.graph_dir, f'readings_at_time_{time_stamp}.png'))
                    plt.close()

                    plt.figure()
                    plt.plot(self.time_buffer, self.intercepts, color='red', label='Intercepts')  # x = sample index
                    plt.xlabel('Time (s)')
                    plt.ylabel('Voltage (V)')
                    plt.title('Battery Intercept (samples)')
                    plt.legend()
                    plt.tight_layout()
                    plt.savefig(os.path.join(self.graph_dir, 'battery_voltage_plot.png'))
                    plt.close()
                except Exception as e:
                    self.get_logger().error(f"Error calculating linear regression for graphing: {e}")
                    return
        

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()