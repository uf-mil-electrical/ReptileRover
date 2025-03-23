import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from first_auto_drive.sonar.sonar_driver import Sonar
import time

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'sonar/data', 10)

        self.subscription = self.create_subscription(String, "gpio/write", self.gpio_write_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_cb)

        # here are the lines that determine how many sonar and pin outs
        self.sonars = [
            Sonar(20, 21),
        ]

    def gpio_write_callback(self, msg):
        msg = msg.data
        msg = msg.split()
        pin = int(msg[0])
        level = int(msg[1])
        self.sonars[0].gpio_write(pin, level)

    def timer_cb(self):
        measurements = []

        for sonar in self.sonars:
            meas = sonar.single_measure()
            if meas == -1:
                continue
            measurements.append(meas)
            # time.sleep(2 * 10**-3) # delay to help w noise NOT need when only one sonar

        msg = Float64MultiArray()
        msg.data = measurements
        #print(measurements)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    my_node = SonarNode()
    
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

