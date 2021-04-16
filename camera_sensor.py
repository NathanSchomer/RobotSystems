from picarx_improved import PicarX
from time import time
from vilib import Vilib


class Sensor:

    def __init__(self, picar=PicarX(), sensitivity=0.5):
        self.sensitivity = sensitivity
        self.picar = picar
        Vilib.camera_start(True)
        Vilib.color_detect_switch(True)
        Vilib.detect_color_name('blue')

    def read():
        return


class SensorProcessing:

    def __init__(self):
        pass

    def process(self, data):
        return


class Controller:

    def __init__(self):
        pass

    def steer(self, cmd):
        pass


if __name__ == "__main__":

    sensitivity = 0.99
    scale = 200
    max_time = 10
    speed = 20

    car = PicarX()
    sensor = Sensor(picar=car)
    proc = SensorProcessing()
    control = Controller(car)

    # car.forward(speed)
    t = time()
    while time() - t < max_time:
        vals = sensor.read()
        # dir_val = proc.process(vals)
        # angle = control.steer(dir_val)
    car.stop()
