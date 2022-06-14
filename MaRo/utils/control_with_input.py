#!/usr/bin/env python

"""
Node for control PCA9685 using AckermannDriveStamped msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import asyncio

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
        self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1
    ):

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
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        pulse_diff = pulse - self.prev_pulse

        if abs(pulse_diff) > 40:
            if pulse_diff > 0:
                pulse += 0.7 * pulse_diff
            else:
                pulse -= 0.7 * pulse_diff

        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)


class Vehicle(object):
    def __init__(self, name="donkey_ros"):

        self._throttle = PCA9685(channel=0, busnum=1)
        print("Throttle Controller Awaked!!")

        self._steering_servo = PCA9685(channel=1, busnum=1)
        print("Steering Controller Awaked!!")

        self._loop = asyncio.get_event_loop()

        self._name = name
        self.speed_pulse = 350
        self.steering_pulse = 400

    # TODO : Calibration
    def control(self): # , speed, steering_angle
        # self.speed_pulse = speed
        # self.steering_pulse = steering_angle

        print(
            "speed_pulse : "
            + str(self.speed_pulse)
            + " / "
            + "steering_pulse : "
            + str(self.steering_pulse)
        )

        self._throttle.run(self.speed_pulse)
        self._steering_servo.run(self.steering_pulse)

    def get_input(self):

        self.speed_pulse = int(input(f"Current Th : {self.speed_pulse} Throttle : "))
        self.steering_pulse = int(input(f"Currnet Angle : {self.steering_pulse} Angle : "))

    async def control_loop(self):
        while True:
            await self._loop.run_in_executor(None, self.control)

    async def input_loop(self):
        while True:
            await self._loop.run_in_executor(None, self.get_input)

    def run(self):
        try:
            asyncio.ensure_future(self.control_loop())
            asyncio.ensure_future(self.input_loop())
            self._loop.run_forever()
        except Exception as e:
            print(e)
        finally:
            print("Done...")

if __name__ == "__main__":

    myCar = Vehicle("donkey_ros")
    myCar.run()

    # while True:
    #     # th = int(input("Throttle : "))
    #     an = int(input("Angle : "))
    #     # myCar.control(th, 350)
    #     myCar.control(350, an)
    #     # myCar.control(th, an)
