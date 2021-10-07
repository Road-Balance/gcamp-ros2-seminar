# Copyright 2021 @RoadBalance
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This is an third example code for ROS 2 rclpy node programming.

Let's learn about those things.

Implement Example 3 with ROS 2 Node Composition.
"""
import time

from rccontrol_msgs.msg import RCControl
import rclpy
from rclpy.node import Node


class PCA9685:
    """PCA9685 capsulation (Not a ROS thing).

    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
        self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1
    ):
        """Get handler for PCA9685 i2c motor driver."""
        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus

        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = 340
        self.prev_pulse = 340
        self.running = True

    def set_pwm(self, pulse):
        """Do pulse send into actial PCA driver."""
        self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        """Run API for this Class"."""
        # pulse_diff = pulse - self.prev_pulse

        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        """Pulse variable Setter."""
        self.pulse = pulse

    def update(self):
        """Do update its state."""
        while self.running:
            self.set_pulse(self.pulse)


class RCSignalSub(Node):
    """Second Node Class.

    Just print log periodically.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('rc_control_node')
        queue_size = 10

        self._throttle = PCA9685(channel=0, busnum=1)
        self.get_logger().info('Throttle Controller Awaked!!')

        self._steering_servo = PCA9685(channel=1, busnum=1)
        self.get_logger().info('Steering Controller Awaked!!')

        self._subscriber = self.create_subscription(
            RCControl, 'spider_control', self.sub_callback, queue_size
        )
        self._subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        """Timer will whenever message got data from anywhere."""
        speed_pulse = msg.throttle
        steering_pulse = msg.steering

        print(
            'speed_pulse : '
            + str(speed_pulse)
            + ' / '
            + 'steering_pulse : '
            + str(steering_pulse)
        )

        self._throttle.run(speed_pulse)
        self._steering_servo.run(steering_pulse)


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    rc_subscriber = RCSignalSub()

    rclpy.spin(rc_subscriber)

    rc_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
