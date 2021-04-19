#!/usr/bin/env python

"""
sim_ezblock.py: Sim EzBlock implements functions
    required for offline PiCar code development
"""
__author__ = "Nathan Schomer"
__email__ = "nathan@nathanschomer.com"


class Servo():

    def __init__(self, pwm):
        return

    def angle(self, angle):
        return


class PWM():

    def __init__(self, channel, debug="critical"):
        return

    def i2c_write(self, reg, value):
        return

    def freq(self, *freq):
        return 1

    def prescaler(self, *prescaler):
        return 1

    def period(self, *arr):
        return

    def pulse_width(self, *pulse_width):
        return

    def pulse_width_percent(self, *pulse_width_percent):
        return


class Pin():

    def __init__(self, *value):
        return

    def check_board_type(self):
        pass

    def init(self, mode, pull):
        pass

    def dict(self, *_dict):
        if len(_dict) == 0:
            return self._dict
        else:
            if isinstance(_dict, dict):
                self._dict = _dict
            else:
                self._error(
                    'argument should be a pin dictionary like {"my pin": ezblock.Pin.cpu.GPIO17}, not %s' % _dict)

    def __call__(self, value):
        return None

    def value(self, *value):
        return 0

    def on(self):
        return True

    def off(self):
        return True

    def high(self):
        return True

    def low(self):
        return True

    def mode(self, *value):
        return (0, 0)

    def pull(self, *value):
        return True

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        return "GPI0N"

    def names(self):
        return ["name", "board_name"]

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4  = 4
        GPIO5  = 5
        GPIO6  = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass

class ADC():

    def __init__(self, chn):
        pass

    def read(self):
        return 0

    def read_voltage(self):
        return 0
