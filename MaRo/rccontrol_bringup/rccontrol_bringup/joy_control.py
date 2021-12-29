import rclpy
from rclpy.node import Node

from rccontrol_msgs.msg import RCControl
from sensor_msgs.msg import Joy

import rccontrol_bringup.Cablibration as calib

# X mode

class JoyControler(Node):

    def __init__(self):
        super().__init__('joy_rc_control_node')

        self.DEFAULT_THROTTLE_VAL = calib.DEFAULT_THROTTLE_VAL
        self.DEFAULT_STEERING_VAL = calib.DEFAULT_STEERING_VAL

        self.MAX_ACCELL_VEL = calib.MAX_ACCELL_VEL
        self.MIN_ACCELL_VEL = calib.MIN_ACCELL_VEL

        self.MAX_STEERING_VEL = calib.MAX_STEERING_VEL
        self.MIN_STEERING_VEL = calib.MIN_STEERING_VEL

        self.create_timer(0.01, self.timer_callback)

        self._subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self._publisher = self.create_publisher(RCControl, 'rc_control', 10)

        self._pub_msg = RCControl()
        self._is_back = False
        self._back_count = 0

    def timer_callback(self):

        if self._back_count > 10:
            self._back_count -= 1
            self._pub_msg.throttle = self.DEFAULT_THROTTLE_VAL - 30
        elif self._back_count > 0:
            self._back_count -= 1
            self._pub_msg.throttle = self.DEFAULT_THROTTLE_VAL

        self._publisher.publish(self._pub_msg)

    def joy_callback(self, joy_msg):

        joy_throttle_val = joy_msg.axes[1]
        joy_steering_val = joy_msg.axes[3]

        if joy_throttle_val > 0.0:
            self._pub_msg.throttle = int(self.DEFAULT_THROTTLE_VAL + joy_throttle_val * (self.MAX_ACCELL_VEL - self.DEFAULT_THROTTLE_VAL))
            self._is_back = False
        else:
            self._pub_msg.throttle = int(self.DEFAULT_THROTTLE_VAL + joy_throttle_val * (self.DEFAULT_THROTTLE_VAL - self.MIN_ACCELL_VEL))
            if self._is_back == False:
                self._back_count = 20

        if joy_steering_val > 0.0:
            self._pub_msg.steering = int(self.DEFAULT_STEERING_VAL + joy_steering_val * (self.MAX_STEERING_VEL - self.DEFAULT_STEERING_VAL))
        else:
            self._pub_msg.steering = int(self.DEFAULT_STEERING_VAL + joy_steering_val * (self.DEFAULT_STEERING_VAL - self.MIN_STEERING_VEL))

def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    joy_ctrl_node = JoyControler()

    rclpy.spin(joy_ctrl_node)
    joy_ctrl_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    """main function"""
    main()
