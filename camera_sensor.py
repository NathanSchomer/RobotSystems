from picarx_improved import PicarX
from time import time, sleep
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np


class Sensor:

    def __init__(self, picar=PicarX(), sensitivity=0.5):
        self.sensitivity = sensitivity
        self.picar = picar
        self.camera = PiCamera()
        sleep(0.1)

    def read(self):
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.capture(self.rawCapture, format="bgr")
        img = self.rawCapture.array
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([30, 40, 0])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        return mask


class SensorProcessing:

    def __init__(self):
        pass

    def sum_row(self, row):

        net_dist = 0
        thresh = 250

        center_idx = row.shape[0] / 2

        idxs = np.argwhere(row > thresh)[:, 0]

        if len(idxs) == 0:
            return 0
        
        for idx in [min(idxs), max(idxs)]:
            net_dist += idx - center_idx

        return net_dist/2


    def process(self, img):

        # canny edge detection
        img = cv2.Canny(img, 200, 400)

        # delete top half of image then flip
        img = img[img.shape[0]//2:, :]
        img = cv2.flip(img, 0)

        # this essentially calculates distance from center for each row
        dists = [self.sum_row(img[n, :]) for n in range(img.shape[0])]

        mean_dist = sum(dists) / len(dists)
        return img, mean_dist / (img.shape[1]//2)


class Controller:

    def __init__(self, picar, scale=10):
        self.picar = picar
        self.scale = scale

    def steer(self, direction):
        angle = self.scale * direction
        self.picar.set_dir_servo_angle(angle)
        return angle


if __name__ == "__main__":

    sensitivity = 0.99
    scale = 40
    max_time = 30
    speed = 20

    car = PicarX()
    sensor = Sensor(picar=car)
    proc = SensorProcessing()
    control = Controller(car, scale=scale)

    car.forward(speed)
    t = time()
    while time() - t < max_time:
        img = sensor.read()
        mask, dir_val = proc.process(img)
        print(dir_val)
        angle = control.steer(dir_val)
        print(angle)
    car.stop()
