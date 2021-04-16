from picarx_improved import PicarX
from time import time


class Sensor:

    def __init__(self, picar=PicarX()):
        self.picar = picar
        self.sensors = {"left": picar.S0,
                        "middle": picar.S1,
                        "right": picar.S2}

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

    @staticmethod
    def calc_deltas(sensor_vals):

        middle_idx = len(sensor_vals) / 2.0
        deltas = [0] * (len(sensor_vals) - len(sensor_vals) % 2)

        for n, val in enumerate(sensor_vals):
            if n < middle_idx:
                deltas[n] = sensor_vals[n+1] - val
            elif n > middle_idx:
                deltas[n-1] = sensor_vals[n-1] - val
        return deltas

    def process(self, sensor_vals):
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

    def steer(self, direction):
        angle = self.scale * direction
        self.picar.set_dir_servo_angle(angle)
        return angle


if __name__ == "__main__":

    sensitivity = 0.99
    polarity = 0
    scale = 200
    max_time = 10
    speed = 20

    car = PicarX()
    sensor = Sensor(picar=car)
    proc = SensorProcessing(sensitivity=sensitivity,
                            polarity=polarity)
    control = Controller(picar=car,
                         scale=scale)

    car.forward(speed)
    t = time()
    while time() - t < max_time:
        vals = sensor.read()
        dir_val = proc.process(vals)
        angle = control.steer(dir_val)
    car.stop()
