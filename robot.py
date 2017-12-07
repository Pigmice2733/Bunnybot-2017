#!/usr/bin/env python3

import wpilib
from ctre.cantalon import CANTalon
from magicbot import MagicRobot

from components.bunnygrabber import BunnyGrabber
from components.drivetrain import Drivetrain
from components.flipper import Flipper
from components.arm import Arm


class Robot(MagicRobot):

    drivetrain = Drivetrain
    bunnygrabber = BunnyGrabber
    flipper = Flipper
    arm = Arm

    def createObjects(self):
        # Drivetrain
        self.front_left_motor = CANTalon(1)
        self.back_left_motor = CANTalon(2)
        self.front_right_motor = CANTalon(3)
        self.back_right_motor = CANTalon(4)

        self.back_left_motor.setControlMode(CANTalon.ControlMode.Follower)
        self.back_right_motor.setControlMode(CANTalon.ControlMode.Follower)

        self.back_left_motor.set(self.front_left_motor.getDeviceID())
        self.back_right_motor.set(self.front_right_motor.getDeviceID())

        self.robot_drive = wpilib.RobotDrive(self.front_left_motor,
                                             self.front_right_motor)

        # Arm
        self.extended_limit_switch = wpilib.DigitalInput(0)
        self.retracted_limit_switch = wpilib.DigitalInput(1)
        self.arm_motor = CANTalon(5)

        self.drivetrain_gyro = wpilib.ADXRS450_Gyro()
        self.bunny_motor = wpilib.Talon(0)

        self.flipper_motor = wpilib.Talon(1)

        # Joysticks
        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

    def teleopPeriodic(self):
        self.drivetrain.turn_at(
            self.drive_joystick.getRawAxis(0), squaredInputs=True)
        self.drivetrain.forward_at(self.drive_joystick.getRawAxis(1))

        if self.drive_joystick.getRawButton(4):
            self.bunny_grabber.set_forward()
        elif self.drive_joystick.getRawButton(5):
            self.bunny_grabber.set_backward()

        if self.drive_joystick.getRawButton(1):
            self.flipper.turn_on()
        else:
            self.flipper.turn_off()

        if self.drive_joystick.getRawButton(11):
            self.arm.extend()
        else:
            self.arm.retract()


if __name__ == '__main__':
    wpilib.run(Robot)
