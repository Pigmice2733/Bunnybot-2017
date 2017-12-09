import ctre
import wpilib
import enum


class Action(enum.Enum):
    Intake = 1
    Outtake = -1
    Stop = 0


class Intake:
    """Bunny intake"""
    intake_motor = ctre.CANTalon
    intake_pdp_channel = int
    pdp = wpilib.PowerDistributionPanel
    # Max current in amps to draw before stopping the motor
    max_current = 6
    release_bunny = False
    holding_bunny = False

    def spit_bunny(self):
        """Spit out a bunny if one is being held"""
        self.release_bunny = True

    def execute(self):
        if self.holding_bunny:
            if self.release_bunny:
                self.intake_motor.set(-1.0)
                self.holding_bunny = False
            else:
                self.intake_motor.set(0.1)
        else:
            self.intake_motor.set(0.7)
            if self.pdp.getCurrent(self.intake_pdp_channel) > self.max_current:
                self.holding_bunny = True
        self.release_bunny = False
