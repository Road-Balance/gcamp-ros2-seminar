from rccontrol_msgs.msg import RCControl
import rclpy
from rclpy.node import Node

import sys, select, termios, tty


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
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyBoardRCController(Node):

    DEFAULT_THROTTLE_VAL = 350
    DEFAULT_STEERING_VAL = 350

    ROSCAR_MAX_ACCELL_VEL = 400
    ROSCAR_MAX_STEERING_VEL = 400

    ROSCAR_MIN_ACCELL_VEL = 200
    ROSCAR_MIN_STEERING_VEL = 200

    def __init__(self):
        super().__init__('keyboard_rc_control_node')

        self._status = 0
        
        self._valid_key_list = ['w','a','s','d',' ','x']
        self._value_dict = {
            "throttle" : self.DEFAULT_THROTTLE_VAL,
            "steering" : self.DEFAULT_STEERING_VAL,
        }
        
        self._publisher = self.create_publisher(RCControl, 'rc_control', 10)
        self.create_timer(0.05, self.timer_callback)

        self._pub_msg = RCControl()
        
        print(msg)

    def publishMsg(self):
        throttle = self._value_dict["throttle"]
        steering = self._value_dict["steering"]

        print(f"currently:\taccell vel {throttle}\t steering vel {steering}")

        self._pub_msg.throttle = self._value_dict["throttle"]
        self._pub_msg.steering = self._value_dict["steering"]

        self._publisher.publish(self._pub_msg)

    def checkLimits(self):
        self._value_dict["throttle"] = min(max(self.ROSCAR_MIN_ACCELL_VEL, self._value_dict["throttle"]), self.ROSCAR_MAX_ACCELL_VEL)
        self._value_dict["steering"] = min(max(self.ROSCAR_MIN_STEERING_VEL, self._value_dict["steering"]), self.ROSCAR_MAX_STEERING_VEL)

    def setThrottle(self, val):
        self._value_dict["throttle"] += val
        self.checkLimits()

    def setSteering(self, val):
        self._value_dict["steering"] += val
        self.checkLimits()

    def resetAllVal(self):
        self._value_dict["throttle"] = self.DEFAULT_THROTTLE_VAL
        self._value_dict["steering"] = self.DEFAULT_THROTTLE_VAL

    def timer_callback(self):
        key = getKey()
        if key in self._valid_key_list:
            if key == 'w':
                self.setThrottle(+1)
                self.publishMsg()
            elif key == 's' :
                self.setThrottle(-1)
                self.publishMsg()
            elif key == 'a' :
                self.setSteering(-1)
                self.publishMsg()
            elif key == 'd' :
                self.setSteering(+1)
                self.publishMsg()
            elif key == ' ' or key == 's' :
                self.resetAllVal()
                self.publishMsg()
        else:
            if (key == 'q'):
                print("quit...")
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

