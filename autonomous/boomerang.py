from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain


class Boomerang(AutonomousStateMachine):
    MODE_NAME = "Boomerang"
    DEFAULT = True

    drivetrain = Drivetrain

    @state(first=True)
    def forward(self):
        if self.drivetrain.forward(feet=8):
            self.next_state('flip')

    @state
    def flip(self):
        if self.drivetrain.rotate(degrees=180, max_speed=1):
            self.next_state('back')

    @state
    def back(self):
        if self.drivetrain.forward(feet=8):
            self.next_state('flip_again')

    @state
    def flip_again(self):
        if self.drivetrain.rotate(degrees=180, max_speed=1):
            self.done()
