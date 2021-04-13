import picarx_improved as picar


class Sensor:

    def __init__(self):
        self.sensors = {"left": picar.S0,
                        "middle": picar.S1,
                        "right": picar.S2}

    def read(self):
        """
        Poll three ADCs and return their values in a list
        """
        return [val.read() for val in self.sensors.values()]


class SensorProcessing:

    def __init__(self, sensitivity, polarity):
        """
        Args:
            sensitivity (float) : The first parameter
            polarity    (bool)  : The second parameter
        """
        return

    def process(self, sensor_readings):
        """
        Args:
            sensor_readings (list) : list of sensor readings

        Returns:
            float : Robot position relative to line in range [-1, 1]
                        with positive values being to the left of the robot
        """
        return


if __name__ == "__main__":
    sensor = Sensor()
