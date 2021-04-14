from picarx_improved import PicarX


class Sensor:

    def __init__(self, picar=PicarX()):
        self.picar = picar
        self.sensors = {"left": picar.S0,
                        "middle": picar.S1,
                        "right": picar.S2}

    def read(self):
        """
        Poll three ADCs and return their values in a list
        """
        return [val.read() for val in self.sensors.values()]


class SensorProcessing:

    def __init__(self, sensitivity=300, polarity=0):
        """
        Args:
            sensitivity (float) : Edge threshold between line and background
            polarity    (int)   : Must be 0 or 1
        """
        assert polarity in [0, 1],\
            "polarity must be 0 or 1"
        assert sensitivity > 0,\
            "sensitivity must be greater than zero"

        self.sensitivity = sensitivity
        self.polarity = polarity

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
        deltas = [val - sensor_vals[n+1] for n, val in enumerate(sensor_vals[:-1])]
        if self.polarity == 0:
            # move down gradient
        else:
            # move up gradient
        print(dirs)

if __name__ == "__main__":
    
    sensor = Sensor()
    proc = SensorProcessing()

    vals = sensor.read()
    proc.process(vals)
