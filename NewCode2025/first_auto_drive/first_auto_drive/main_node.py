# take the 
# imu
# sonar
# controller and do this
# 1. if the controller gives any about ignore everything else and do that
# 2. if the sonar says 20cm or less, start_turn
# 3. start_turn uses the imu to turn 90 degress and then we go back to step 1

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64MultiArray
from first_auto_drive.wheels.tank_drive_train import TankDriveTrain

class MainNode(Node):
    def __init__(self):
        # motors for later
        self.tank_drive_train = TankDriveTrain()

        # topic data stored here
        self.joy_data = [0, 0] # gonna be [x, y] on the joystick
        self.imu_data = 0 # this will be a float that is like compass angle
        self.sonar_data = [] # distances from each sonar [10, 12, 2] means sonar1 gives 10 cm sonar 2 reads 12 cm etc.

        # ros stuff
        super().__init__('main_node')

        self.subscription = self.create_subscription( Joy, 'joy', self.joy_callback, 10)
        self.subscription = self.create_subscription( Imu, 'imu/data', self.imu_callback, 10)
        self.subscription = self.create_subscription( Float64MultiArray, 'sonar/data', self.sonar_callback, 10)
        self.subscription

        self.timer = self.create_timer(0.1, self.timer_callback)

    def joy_callback(self, msg):
        # The Joy message contains axes and buttons info from the joystick
        self.joy_data[0] = msg.axes[0]
        self.joy_data[1] = msg.axes[1]

    def sonar_callback(self, msg):
        # just a list of distances detected by sonar
        self.sonar_data = list(msg.data)

    def imu_callback(self, msg):
        pass
        # TODO

    def controller_to_motors(self):
        x_axis = self.joy_data[0]
        y_axis = self.joy_data[1]

        if abs(x_axis) > abs(y_axis):
            if x_axis > 0:
                # print("turn left")
                self.tank_drive_train.left(x_axis)
            else:
                # print("turn right")
                self.tank_drive_train.right(x_axis)
        else:
            # move forwards with y_axis speed
            if y_axis > 0:
                # print("go fowards")
                self.tank_drive_train.right(y_axis)
            else:
                # print("go back")
                self.tank_drive_train.right(y_axis)

    def timer_callback(self):
        # first, should we just use controller
        controller_being_used = not (self.joy_data[0] == 0 and self.joy_data[1] == 0)
        if controller_being_used:
            self.controller_to_motors()
            return

        # second, is there something in front of us
        we_need_to_turn = False
        sonar_thats_to_close = -1
        for i, reading in enumerate(self.sonar_data):
            if reading <20:
                we_need_to_turn == True
                sonar_thats_to_close = i
                break

        if we_need_to_turn:
            pass
            return

        # just keep going forwards
        self.tank_drive_train.forward(0.75)


def main(args=None):
    rclpy.init(args=args)

    main_node = MainNode()

    rclpy.spin(main_node)

    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

