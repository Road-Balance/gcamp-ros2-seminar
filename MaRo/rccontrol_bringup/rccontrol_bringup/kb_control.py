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
KeyBoard Based RC Control Node.

Before Running this node, you must calibrate your hardware with rqt_rc_steering GUI.
Then, Put your values into KeyBoardRCController's Class Variables.

While Running this node, Keyboard inputs will be mapped into RCControl,
and then published through /rc_control topic.
"""
import select
import sys
import termios
import tty
import time

from rccontrol_msgs.msg import RCControl
import rclpy
from rclpy.node import Node


msg = """
Control Your ROS 2 RC CAR!
---------------------------
Moving around:
\tW

A\tS\tD

\tX

w/x : increase/decrease PWM throttle value
a/d : increase/decrease PWM steering value

space key : force stop

"q" to quit
"""

e = """
Communications Failed
"""

settings = termios.tcgetattr(sys.stdin)


def getKey():
    """
    Run every Keyboard Interrupts.

    Parse KB value into readable "str" type
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyBoardRCController(Node):
    """Get KB input and parse into ROS 2 Control Msg Type."""

    
    DEFAULT_THROTTLE_VAL = 350
    DEFAULT_STEERING_VAL = 380

    MAX_ACCELL_VEL = 420
    MIN_ACCELL_VEL = 300

    MAX_STEERING_VEL = 430
    MIN_STEERING_VEL = 300

    FORWARD_VEL = 381
    BACKWARD_VEL = 320

    LEFT_VEL = 300
    RIGHT_VEL = 430

    def __init__(self):
        """
        Create KeyBoard value parsing loop with 0.05s periods.

        Fill in Custom ROS 2 Topic Msg Type (=RCControl) with KB inputs,
        then publisher them through /rc_control topic.
        """
        super().__init__('keyboard_rc_control_node')

        self._status = 0

        self._valid_key_list = ['w', 'a', 's', 'd', ' ', 'x']
        self._value_dict = {
            'throttle': self.DEFAULT_THROTTLE_VAL,
            'steering': self.DEFAULT_STEERING_VAL,
        }

        self._publisher = self.create_publisher(RCControl, 'rc_control', 10)
        self.create_timer(0.05, self.timer_callback)

        self._pub_msg = RCControl()

        print(msg)

    def publishMsg(self):
        """Publish ROS 2 topic, Run this in every loops."""
        throttle = self._value_dict['throttle']
        steering = self._value_dict['steering']

        print(f'currently:\taccell vel {throttle}\t steering vel {steering}')

        self._pub_msg.throttle = self._value_dict['throttle']
        self._pub_msg.steering = self._value_dict['steering']

        self._publisher.publish(self._pub_msg)

    def checkLimits(self):
        """Control values should be in valid range."""
        self._value_dict['throttle'] = min(
            max(self.MIN_ACCELL_VEL, self._value_dict['throttle']),
            self.MAX_ACCELL_VEL
        )
        self._value_dict['steering'] = min(
            max(self.MIN_STEERING_VEL, self._value_dict['steering']),
            self.MAX_STEERING_VEL
        )

    def setThrottle(self, val):
        """Setter for throttle value. Do update Dict type Method Variable."""
        self._value_dict['throttle'] = val
        self.checkLimits()

    def setSteering(self, val):
        """Setter for steering value. Do update Dict type Method Variable."""
        self._value_dict['steering'] += val
        self.checkLimits()

    def setLeft(self):
        current_steering = self._value_dict['steering']
        for i in range(current_steering, self.LEFT_VEL - 1, -1):
            self._value_dict['steering'] = i
            self.publishMsg()

    def setRight(self):
        current_steering = self._value_dict['steering']
        for i in range(current_steering, self.RIGHT_VEL + 1, +1):
            self._value_dict['steering'] = i
            self.publishMsg()

    def resetAllVal(self):
        """Reset All control values to initial one."""
        self._value_dict['throttle'] = self.DEFAULT_THROTTLE_VAL
        self._value_dict['steering'] = self.DEFAULT_STEERING_VAL

    def timer_callback(self):
        """Get KB value then Run logic for increasing/decreasing control offset."""
        key = getKey()
        if key in self._valid_key_list:
            if key == 'w':
                self.setThrottle(self.FORWARD_VEL)
                self.publishMsg()
            elif key == 's':
                print("fuck")
                for i in range(100):
                    self.setThrottle(300)
                    self.publishMsg()
                # time.sleep(0.1)
                for i in range(100):
                    self.setThrottle(self.DEFAULT_THROTTLE_VAL)
                    self.publishMsg()
                # time.sleep(0.2)

                self.setThrottle(self.BACKWARD_VEL)
                self.publishMsg()
            elif key == 'a':
                # self.setLeft()
                self.setSteering(+10)
                self.publishMsg()
            elif key == 'd':
                # self.setRight()
                self.setSteering(-10)
                self.publishMsg()
            elif key == ' ' or key == 's':
                self.resetAllVal()
                self.publishMsg()
        else:
            if (key == 'q'):
                print('quit...')
                quit()

            self._status += 1

        if self._status == 100:
            print(msg)
            self._status = 0


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    kb_ctl_node = KeyBoardRCController()
    rclpy.spin(kb_ctl_node)
    kb_ctl_node.destroy_node()

    rclpy.shutdown()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    """main function"""
    main()
