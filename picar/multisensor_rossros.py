from picarx_improved import PicarX
import signal
from rossros import Bus, Consumer, Producer, runConcurrently, ConsumerProducer
from ezblock import Pin
from ezblock import Ultrasonic


# globals
picar = PicarX()
the_terminator = None


def read_sensor():
    sensors = {"left": picar.S0,
               "middle": picar.S1,
               "right": picar.S2}
    return [val.read() for val in sensors.values()]


def calc_deltas(sensor_vals):

    middle_idx = len(sensor_vals) / 2.0
    deltas = [0] * (len(sensor_vals) - len(sensor_vals) % 2)

    for n, val in enumerate(sensor_vals):
        if n < middle_idx:
            deltas[n] = sensor_vals[n+1] - val
        elif n > middle_idx:
            deltas[n-1] = sensor_vals[n-1] - val
    return deltas


def proc_grayscale_sensor(sensor_vals):

    # constants
    max_val = 1720
    sensitivity = 0.99
    polarity = 0

    deltas = calc_deltas(sensor_vals)

    if polarity == 1:
        deltas = [-1*d for d in deltas]

    left = sum(deltas[:len(deltas)//2])
    right = sum(deltas[len(deltas)//2:])

    n_half = len(deltas)//2

    thresh = (1 - sensitivity) * (n_half * max_val)

    dir_value = 0

    if left > thresh:
        dir_value -= (left-thresh) / (max_val-thresh)

    if right > thresh:
        dir_value += (right-thresh) / (max_val-thresh)

    return dir_value


def control(direction):
    global picar
    scale = 100
    picar.set_dir_servo_angle(scale*direction)


def read_ultrasonic():
    pin_D0=Pin("D0")
    pin_D1=Pin("D1")
    return Ultrasonic(pin_D0, pin_D1).read()

def proc_ultrasonic(sensor_val):
    thresh = 10
    stop = False
    if sensor_val < thresh:
        stop = True
    return stop


def control_ultrasonic(stop):
    global picar, the_terminator
    speed = 20
    if stop:
        picar.stop()
    else:
        picar.forward(speed)


def sigint_handler():
    global the_terminator
    the_terminator.set_message(True)


if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)

    sensitivity = 0.99
    polarity = 0
    scale = 200

    # grayscale sensor busses
    sensor_values_bus = Bus(initial_message=[0, 0, 0],
                            name="sensor values bus")
    interpreter_bus = Bus(initial_message=0,
                          name="interpreter bus")

    # ultrasonic sensor busses
    ultra_sensor_bus = Bus(initial_message=0,
                           name="ultrasonic sensor bus")
    ultra_interpreter_bus = Bus(initial_message=False,
                                name="ultrasonic interpreter bus")

    # tell threads when to die
    the_terminator = Bus(initial_message=False,
                         name="Ill be back")

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    # grayscale sensor threads
    gray_producer = Producer(read_sensor,
                             output_busses=sensor_values_bus,
                             delay=0.09,
                             name="Grayscale Sensor Producer")
    gray_proc = ConsumerProducer(proc_grayscale_sensor,
                                 input_busses=sensor_values_bus,
                                 output_busses=interpreter_bus,
                                 delay=0.1,
                                 name="Grayscale Sensor Processing")
    controller = Consumer(control,
                          input_busses=interpreter_bus,
                          delay=0.1,
                          name="Steering Controller")

    # ultrasonic sensor threads
    ultra_producer = Producer(read_ultrasonic,
                              output_busses=ultra_sensor_bus,
                              delay=0.01,
                              name="Grayscale Sensor Producer")
    ultra_proc = ConsumerProducer(proc_ultrasonic,
                                  input_busses=ultra_sensor_bus,
                                  output_busses=ultra_interpreter_bus,
                                  delay=0.05,
                                  name="Grayscale Sensor Processing")
    ultra_control = Consumer(control_ultrasonic,
                             input_busses=ultra_interpreter_bus,
                             delay=0.05,
                             name="Steering Controller")

    thread_list = [gray_producer, gray_proc, controller, ultra_producer, ultra_proc, ultra_control]

    # lets get this party started
    runConcurrently(thread_list)
    picar.stop()
