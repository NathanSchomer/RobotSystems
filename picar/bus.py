from time import time


class Bus:

    def __init__(self, msg_type=None):
        self._type = msg_type
        self._message = None
        self._time = None

    def read(self):
        return self._message, self._time

    def write(self, msg):

        if self._type is not None:
            assert type(msg) is self._type,\
                "Message must be of type {}".format(self._type)

        self._message = msg
        self._time = time()


if __name__ == "__main__":

    # create bus with msg type of int
    bus = Bus(int)

    # write message to bus
    msg = 10
    bus.write(msg)

    # read message from bus
    msg, t = bus.read()
    print("Message: {}\nTimestamp: {}".format(msg, t))

    bus.write(float(10))
