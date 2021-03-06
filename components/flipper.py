import wpilib


class Flipper:
    """this flips a bucket with an arm that has a spinning motor"""

    flipper_motor = wpilib.Talon
    spinning = False

    def turn_on(self):
        self.spinning = True

    def turn_off(self):
        self.spinning = False

    def execute(self):
        if self.spinning:
            self.flipper_motor.set(0.2)
        else:
            self.flipper_motor.set(0)
