from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain


class Rotate(AutonomousStateMachine):
    MODE_NAME = "Rotate"
    DEFAULT = True

    drivetrain = Drivetrain

    @state(first=True)
    def rotate(self):
        if self.drivetrain.rotate(degrees=-90):
            self.done()
