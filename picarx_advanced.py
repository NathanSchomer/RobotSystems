from picarx_improved import PicarX
from time import sleep
import sys, tty, termios


def get_char(prompt=None):
    """
    NOT MINE
    Courtesy of https://code.activestate.com/recipes/134892/
    """
    if prompt:
        print(prompt)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class PicarX_Advanced(PicarX):

    def __init__(self):
        super().__init__()
        self.angle = 0
        self.speed = 0
        return

    def parallel_park(self, direction='left'):

        if direction == 'left':
            direction = -1
        else:
            direction = 1

        # developed via the highly scientific "guess and check" methodology
        self.set_dir_servo_angle(40*direction)
        self.backward(55)
        sleep(0.86)
        self.set_dir_servo_angle(-40*direction)
        sleep(0.93)
        self.set_dir_servo_angle(0)
        self.forward(55, 0.55)
        self.stop()

    def austin_powers(self):
        for _ in range(5):
            self.angle(10)
            self.forward(50)
            sleep(0.5)
            self.stop()
            sleep(0.5)
            self.angle(-10)
            self.backward(50)
            sleep(0.5)
            self.stop()
            sleep(0.5)

    def k_turn(self, direction='left'):

        if direction == 'left':
            direction = -1
        else:
            direction = 1

        self.set_dir_servo_angle(40*direction)
        self.backward(55)
        sleep(1)
        self.set_dir_servo_angle(-40*direction)
        self.forward(55)
        sleep(1)
        self.stop()

    def stop(self):
        """
        Overloads super().stop() to also zero-out angle and power
        """
        super().stop()
        self.angle = 0
        self.power = 0

    @staticmethod
    def prompt_left_or_right():
        prompt = '\nChoose direction:\n' +\
                 '\tL: left\n' +\
                 '\tR: right\n' +\
                 '\tX: exit\n\n' +\
                 'Choice: '

        choice = get_char(prompt).lower()
        if choice == 'l':
            choice = 'left'
        elif choice == 'r':
            choice = 'right'
        else:
            choice = None

        return choice

    def interactive(self):
        prompt = "\x1b[2J\x1b[H" +\
                 "Use WASD to control the robot" +\
                 "or select one of the following actions:\n" +\
                 "\tP: parallel park\n\tK: k-turn\n\te: stop\n" +\
                 "Use esc or X to exit\n\n"

        print(prompt)
        choice = get_char().lower()

        while choice != 'x':

            if choice == 'w':
                self.speed += 10
            elif choice == 's':
                self.speed -= 10
            elif choice == 'a':
                self.angle -= 5
            elif choice == 'd':
                self.angle += 5
            elif choice == 'p':
                direction = self.prompt_left_or_right()
                if direction is None:
                    continue
                self.parallel_park(direction=direction)
                self.stop()
            elif choice == 'k':
                direction = self.prompt_left_or_right()
                if direction is None:
                    continue
                self.k_turn(direction=direction)
                self.stop()
            elif choice == 'e':
                self.angle = 0
                self.speed = 0
                self.stop()
            elif choice == 'q':
                self.austin_powers()
                self.angle = 0
                self.speed = 0
                self.stop()

            self.set_dir_servo_angle(self.angle)
            self.forward(self.speed)

            print(prompt)
            choice = get_char().lower()


if __name__ == "__main__":

    pca = PicarX_Advanced()
    pca.interactive()
