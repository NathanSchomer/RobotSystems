from picarx_improved import PicarX
from time import time, sleep
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
from threading import Event, Lock
import concurrent.futures
import signal
from bus import Bus


# better way to do this??
_stop_requested = Event()


class Sensor:

    """
    This class is used to retrieve unprocessed images
        from the RaspberryPi camera. Frames are 1280x1024
        with BGR pixel format
    """

    def __init__(self, picar=PicarX(), sensitivity=0.5):
        self.sensitivity = sensitivity
        self.picar = picar
        self.camera = PiCamera()
        sleep(0.1)

    def read_threaded(self, bus: Bus, delay: float, kill_thread: Event):

        while not kill_thread.is_set():
            bus.write(self.read())
            sleep(delay)

    def read(self):
        """
        Return frame from camera
        """
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.capture(self.rawCapture, format="bgr")
        img = self.rawCapture.array
        return img


class SensorProcessing:

    """
    This class is used to determine vehicle offset from
        a blue line in a given image.
    """

    def __init__(self):
        pass

    def process_threaded(self, in_bus: Bus, out_bus: Bus,
                         delay: float, kill_thread: Event):

        while not kill_thread.is_set():
            sensor_vals = in_bus.read()
            if sensor_vals is not None:
                control_val = self.process(sensor_vals)
                out_bus.write(control_val)
            sleep(delay)

    @staticmethod
    def sum_row(row):
        """
        For a provided row of an image, determine center-offset
            of the centroid of the outermost non-zero pixels.
        """

        net_dist = 0
        thresh = 250
        center_idx = row.shape[0] / 2

        # determine indicies with above-threshold values
        idxs = np.argwhere(row > thresh)[:, 0]

        if len(idxs) == 0:
            return 0

        # calculate centroid of outermost above-threshold indicies
        for idx in [min(idxs), max(idxs)]:
            net_dist += idx - center_idx
        ret = net_dist / 2

        return ret

    def process(self, img) -> float:
        """
        For a given image, determine porportional offset from line
            with return value in the range [-1, 1]
        """

        # create mask of blue pixels
        lower_blue = np.array([30, 40, 0])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        upper_blue = np.array([150, 255, 255])
        img = cv2.inRange(hsv, lower_blue, upper_blue)

        # canny edge detection
        img = cv2.Canny(img, 200, 400)

        # delete top half of image then flip
        #   (we only care about pixels closest to vehicle)
        img = img[img.shape[0]//2:, :]
        img = cv2.flip(img, 0)

        # this essentially calculates distance from center for each row
        dists = [self.sum_row(img[n, :]) for n in range(img.shape[0])]

        mean_dist = sum(dists) / len(dists)
        return mean_dist / (img.shape[1]//2)


class Controller:

    def __init__(self, picar, scale=10):
        self.picar = picar
        self.scale = scale

    def steer_threaded(self, bus: Bus,
                       delay: float, kill_thread: Event):

        while not kill_thread.is_set():
            self.steer(bus.read())
            sleep(delay)

    def steer(self, direction):
        angle = self.scale * direction
        self.picar.set_dir_servo_angle(angle)
        return angle


def run_single_thread(car, sensor, proc, control):
    max_time = 10
    car.forward(speed)
    t = time()
    while time() - t < max_time:
        vals = sensor.read()
        dir_val = proc.process(vals)
        control.steer(dir_val)
    car.stop()


def sigint_handler(sig, frame):
    global _stop_requested
    _stop_requested.set()


if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)

    scale = 40
    max_time = 30
    speed = 20

    car = PicarX()
    sensor = Sensor(picar=car)
    proc = SensorProcessing()
    control = Controller(car, scale=scale)

    car.forward(speed)

    # setup busses
    sensor_values_bus = Bus()
    interpreter_bus = Bus()

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor.read_threaded, sensor_values_bus,
                                  sensor_delay, _stop_requested)
        eInterpreter = executor.submit(proc.process_threaded,
                                       sensor_values_bus,
                                       interpreter_bus, interpreter_delay,
                                       _stop_requested)
        eController = executor.submit(control.steer_threaded, interpreter_bus,
                                      control_delay, _stop_requested)
        eSensor.result()

    car.stop()
