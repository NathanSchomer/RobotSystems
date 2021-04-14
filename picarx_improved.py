#!/usr/bin/env python

"""
picarx_improved.py: PicarX improved is a class-based reimplementation
                    of the RaspiCar API provided by SunFounder
"""
__author__ = "Nathan Schomer"
__email__ = "nathan@nathanschomer.com"

import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import time
import atexit
from math import cos, radians, pi

# import skeleton ezblock class if not on raspberry pi
try:
    from ezblock import PWM, ADC, Servo, Pin
    from ezblock import __reset_mcu__
    __reset_mcu__()
    time.sleep(0.01)
except ImportError:
    print("This computer does nto appear to be a PiCar-X system " +
          "(/opt/ezblock is not present). Shadowing hardware calls " +
          "with substitute functions")
    from sim_ezblock import PWM, ADC, Servo, Pin

# setup logging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
                    datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class PicarX:

    def __init__(self):

        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.02

        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")

        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        self.Servo_dir_flag = 1
        self.dir_cal_value = 6.5
        self.cam_cal_value_1 = 0
        self.cam_cal_value_2 = 0
        self.motor_direction_pins = [self.left_rear_dir_pin,
                                     self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin,
                                 self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        atexit.register(self.stop_motors)

    @log_on_start(logging.DEBUG, "stop_motors(): starting")
    @log_on_error(logging.DEBUG, "stop_motors(): error")
    @log_on_end(logging.DEBUG, "stop_motors(): end")
    def stop_motors(self):
        logging.debug("stop_motors()")
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def set_motor_speed(self, motor, speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1*self.cali_dir_value[motor]

    def dir_servo_angle_calibration(self, value):
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
        # dir_servo_pin.angle(self.dir_cal_value)

    def set_dir_servo_angle(self, value):
        self.dir_servo_pin.angle(value+self.dir_cal_value)

    def camera_servo1_angle_calibration(self, value):
        self.cam_cal_value_1 = value
        self.set_camera_servo1_angle(self.cam_cal_value_1)
        # camera_servo_pin1.angle(self.cam_cal_value)

    def camera_servo2_angle_calibration(self, value):
        self.cam_cal_value_2 = value
        self.set_camera_servo2_angle(self.cam_cal_value_2)
        # camera_servo_pin2.angle(self.cam_cal_value)

    def set_camera_servo1_angle(self, value):
        self.camera_servo_pin1.angle(-1 * (value + self.cam_cal_value_1))

    def set_camera_servo2_angle(self, value):
        self.camera_servo_pin2.angle(-1 * (value + self.cam_cal_value_2))

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed, runtime=0):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

        if runtime > 0:
            time.sleep(runtime)
            self.stop()

    def forward(self, speed, runtime=0):

        # constants
        wheelbase_len = 11.0
        half_wheelbase_width = 11.0 / 2.0

        # wheel angle of imaginary center wheel
        theta = radians(self.dir_servo_pin.last_angle)

        speed1 = speed2 = speed

        if theta != 0:
            center_radius = wheelbase_len / cos(pi/2 - theta)
            wheelpath_ratio = (center_radius + half_wheelbase_width) /\
                              (center_radius - half_wheelbase_width)

            # TODO: set maximum wheelpath_ratio value to prevent donuts

            # scale speed of outside wheel when turning
            if theta < 0:
                speed1 *= wheelpath_ratio
            elif theta > 0:
                speed2 *= wheelpath_ratio

        self.set_motor_speed(1, -1*speed1)
        self.set_motor_speed(2, -1*speed2)

        if runtime > 0:
            time.sleep(runtime)
            self.stop()

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def get_distance(self):
        timeout = 0.01
        trig = Pin('D8')
        echo = Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm


def test(picar):
    # dir_servo_angle_calibration(-10)
    picar.set_dir_servo_angle(-40)
    time.sleep(1)
    picar.set_dir_servo_angle(0)
    time.sleep(1)
    # set_motor_speed(1, 1)
    # set_motor_speed(2, 1)
    # camera_servo_pin.angle(0)


if __name__ == "__main__":
    picar = PicarX()
    try:
        picar.dir_servo_angle_calibration(-10)
        while 1:
            test(picar)
    finally:
        picar.stop()
