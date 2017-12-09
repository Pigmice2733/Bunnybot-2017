import math

import ctre
import wpilib

from common.motion_profiles import MotionProfile, ProfileExecutor
from common.pid import PIDCoefficients

# Only the Drivetrain needs to be used outside of this module,
#  if we need to expose something else later we can
__all__ = ["Drivetrain"]


class Drivetrain:
    robot_drive = wpilib.RobotDrive
    gyro = wpilib.ADXRS450_Gyro
    arm_motor = ctre.CANTalon

    def __init__(self):
        self.rotation = 0
        self.forward_speed = 0
        self.gyro_offset = 0.0
        self.profile_executor = None
        self.profile_arguments = None
        self.wheel_circumference_meters = 0.48

    def forward_at(self, speed):
        self.forward_speed = speed

    def turn_at(self, speed, squaredInputs=False):
        self.rotation = speed
        if squaredInputs:
            self.rotation = speed**2 if speed >= 0 else -(speed**2)

    def forward(self, feet=0, inches=0, meters=0, max_speed=1):
        """Use a motion profile and PID control to efficiently
         move the robot the specified distance forward

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile.

        `max_speed` is in units of meters per second"""
        # 1 inch = 0.0254 meters
        # 1 foot = 0.3048 meters
        distance = (inches * 0.0254) + (feet * 0.3048) + meters

        # When called with the same arguments, update executor
        if distance == self.profile_arguments:
            return self._update_executor()

        # When target is switched without calling
        #  cancel_motion_profile, issue warning
        if self.profile_arguments is not None:
            print("Use Drivetrain.cancel_motion_profile to change profile!",
                  "Switching to new profile...")

        self.profile_arguments = distance

        motion_profile = MotionProfile(
            acceleration_time=1,
            deceleration_time=1,
            max_speed=max_speed,
            target_distance=distance)

        coefs = PIDCoefficients(p=1.5, i=0.6, d=0.0)

        # Set current position to zero
        self._reset_encoder_position()

        self.profile_executor = ProfileExecutor(
            coefs, motion_profile,
            lambda: (self._get_encoder_position()/360) * self.wheel_circumference_meters,
            lambda output: self.forward_at(output), 0.01)

        return False

    def backward(self, feet=0, inches=0, meters=0, max_speed=1):
        """Use a motion profile and PID control to efficiently
         move the robot the specified distance backward

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile.

        `max_speed` is in units of meters per second"""
        return self.forward(-feet, -inches, -meters, max_speed)

    def rotate(self, degrees=0, max_speed=5):
        """Use a motion profile and PID control to efficiently
         turn the robot the number of degrees - positive is clockwise,
         negative is counter-clockwise

        Call repeatedly with the same arguments to update, use
         `cancel_motion_profile`, then call again to change target.

        Returns `False` while executing, `True` once done. Continuing
         to update when done will start new profile.

        `max_speed` is in units of degrees per second"""
        radians = degrees * (math.pi / 180)

        # When called with the same arguments, update executor
        if radians == self.profile_arguments:
            return self._update_executor()

        # When target is switched, issue warning
        if self.profile_arguments is not None:
            print("Use Drivetrain.cancel_motion_profile to change profile!",
                  "Switching to new profile...")

        self.profile_arguments = radians

        motion_profile = MotionProfile(
            acceleration_time=0.7,
            deceleration_time=1.4,
            max_speed=max_speed,
            target_distance=radians)

        self._zero_gyro()

        coefs = PIDCoefficients(p=0.85, i=0.3, d=0.08)
        self.profile_executor = ProfileExecutor(
            coefs, motion_profile, lambda: self._get_gyro_angle(),
            lambda output: self.turn_at(-output), 0.003)

        return False

    def reset_motion_profile(self):
        # Resets or cancels motion profile
        self.profile_executor = None
        self.profile_arguments = None

    def execute(self):
        self.robot_drive.arcadeDrive(self.forward_speed, self.rotation)

        self.rotation = 0
        self.forward_speed = 0

    def on_disabled(self):
        self.cancel_motion_profile()

    def _reset_encoder_position(self):
        self.arm_motor.setEncPosition(0)

    def _get_encoder_position(self):
        ticks_per_revolution = 11.3
        return self.arm_motor.getEncPosition() / ticks_per_revolution

    def _zero_gyro(self):
        self.gyro_offset = self.gyro.getAngle()

    def _get_gyro_angle(self):
        return (self.gyro.getAngle() - self.gyro_offset) * (math.pi / 180.0) * 1.013

    def _update_executor(self):
        executor_finished = self.profile_executor.update()
        if executor_finished:
            self.reset_motion_profile()
            return True
        return False
