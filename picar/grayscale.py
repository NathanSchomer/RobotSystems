from picarx_improved import PicarX
from time import time, sleep
from bus import Bus
from threading import Event, Lock
import concurrent.futures
import signal


# better way to do this??
_stop_requested = Event()


class Sensor:

    def __init__(self, picar=PicarX()):
        self.picar = picar
        self.sensors = {"left": picar.S0,
                        "middle": picar.S1,
                        "right": picar.S2}

    def read_threaded(self, bus: Bus, delay: float, kill_thread: Event):

        lock = Lock()

        while not kill_thread.is_set():
            with lock:
                vals = self.read()
            bus.write(vals)
            sleep(delay)

    def read(self):
        """
        Poll three ADCs and return their values in a list
            * sensor values are in range [0, 1720]
        """
        return [val.read() for val in self.sensors.values()]


class SensorProcessing:

    def __init__(self, sensitivity=0.75, polarity=0):
        """
        Args:
            sensitivity (float) : Sensitivity to edge in range [0, 1]
            polarity  (int)   : 0 to follow dark line or 1 to follow light line
        """
        assert 0 <= polarity <= 1,\
            "polarity must be 0 or 1"
        assert sensitivity > 0,\
            "sensitivity must be greater than zero"

        self.sensitivity = sensitivity
        self.polarity = polarity

        self.min_val = 0
        self.max_val = 1720

    def process_threaded(self, in_bus: Bus, out_bus: Bus,
                         delay: float, kill_thread: Event):

        while not kill_thread.is_set():
            sensor_vals = in_bus.read()
            control_val = self.process(sensor_vals)
            out_bus.write(control_val)
            sleep(delay)

    @staticmethod
    def calc_deltas(sensor_vals: list) -> list:

        middle_idx = len(sensor_vals) / 2.0
        deltas = [0] * (len(sensor_vals) - len(sensor_vals) % 2)

        for n, val in enumerate(sensor_vals):
            if n < middle_idx:
                deltas[n] = sensor_vals[n+1] - val
            elif n > middle_idx:
                deltas[n-1] = sensor_vals[n-1] - val
        return deltas

    def process(self, sensor_vals: list) -> float:
        """
        This function is agnostic of the number of sensor_readings.
        Additionally, it is assumed that sensor_readings are in order
        from left to right.

        Args:
            sensor_readings (list) : list of sensor readings

        Returns:
            float : Robot position relative to line in range [-1, 1]
                        with positive values being to the left of the robot
        """

        deltas = self.calc_deltas(sensor_vals)

        if self.polarity == 1:
            deltas = [-1*d for d in deltas]

        left = sum(deltas[:len(deltas)//2])
        right = sum(deltas[len(deltas)//2:])

        n_half = len(deltas)//2

        thresh = (1 - self.sensitivity) * (n_half * self.max_val)

        dir_value = 0

        if left > thresh:
            dir_value -= (left-thresh) / (self.max_val-thresh)

        if right > thresh:
            dir_value += (right-thresh) / (self.max_val-thresh)

        return dir_value


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

    sensitivity = 0.99
    polarity = 0
    scale = 200
    speed = 25

    # setup objects
    car = PicarX()
    sensor = Sensor(picar=car)
    proc = SensorProcessing(sensitivity=sensitivity,
                            polarity=polarity)
    control = Controller(picar=car,
                         scale=scale)

    # setup busses
    sensor_values_bus = Bus(msg_type=list)
    interpreter_bus = Bus(msg_type=float)

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    car.forward(speed)

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
