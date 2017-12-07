import wpilib
import ctre
import enum


class Direction(enum.Enum):
    extend = True
    retract = False


class Arm:
    """Main manipulator arm"""

    arm_motor = ctre.CANTalon
    extended_limit_switch = wpilib.DigitalInput
    retracted_limit_switch = wpilib.DigitalInput
    direction = Direction.retract

    def extend(self):
        self.direction = Direction.extend

    def retract(self):
        self.direction = Direction.retract

    def execute(self):
        motor_speed = 0.0
        if self.direction == Direction.extend:
            if not self.extended_limit_switch.get():
                motor_speed = 0.4
        else:
            if not self.retracted_limit_switch.get():
                motor_speed = -0.2
        self.arm_motor.set(motor_speed)
