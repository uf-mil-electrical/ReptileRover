import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
topic_name = 'imu/data'

class ImuUser(Node):

    def __init__(self):
        # ros stuff
        super().__init__('imu_user')
        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            self.imu_callback,
            10
        )
        self.subscription

        self.client = self.create_client(Trigger, 'imu_trigger_service')

    def imu_callback(self, msg):
            ori = msg.orientation
            roll, pitch, yaw = self.quaternion_to_euler(ori)

            # Convert radians to degrees
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            if input("do it:") == 'yes':
                self.send_trigger_req()

            print(roll_deg)
            print(pitch_deg)
            print(yaw_deg)
            return

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw) in radians

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1 - 2 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def send_trigger_req(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.trigger_response_callback)

    def trigger_response_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Trigger response: {response.message}')
        else:
            self.get_logger().error('Failed to trigger')


def main(args=None):
    # ros2 stuff (setup node start node clenup node)
    rclpy.init(args=args)
    imu_listener = ImuUser()
    rclpy.spin(imu_listener)
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
