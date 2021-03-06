from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain


class Forward(AutonomousStateMachine):
    MODE_NAME = "Forward"

    drivetrain = Drivetrain

    @state(first=True)
    def forward(self):
        if self.drivetrain.forward(feet=10, max_speed=1):
            self.done()
