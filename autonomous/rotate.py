from magicbot import AutonomousStateMachine, state
from components.drivetrain import Drivetrain


class Rotate(AutonomousStateMachine):
    MODE_NAME = "Rotate"

    drivetrain = Drivetrain

    @state(first=True)
    def rotate(self):
        if self.drivetrain.rotate(degrees=-90):     
            '''This tells us that if the robot rotates 90 degrees, the state is done.
            it calls on the drivetrain and it's code for "rotate", then if it can run it, the state ends.'''
            self.done()
