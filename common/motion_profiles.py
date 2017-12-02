"""Provide motion profiles for smooth and efficient motion"""

import math
from typing import Callable

import wpilib

from common.pid import PIDCoefficients, PIDController


class MotionProfile:
    """Motion profile representing a specific desired move"""

    def __init__(self, acceleration_time: float, deceleration_time: float,
                 max_speed: float, target_distance: float):
        """Create a motion profile to efficiently travel `target_distance`

        `acceleration_time`: Time it takes for robot to accelerate from rest
         to maximum speed.
        `deceleration_time`: Time it takes for robot to decelerate from
         maximum speed to rest.
        `max_speed`: Robot's maximum speed.
        `target_distance`: The distance the robot should travel using the
         motion profile.

        ** Note: The units don't matter, as long as they are all the same.**
        """
        # Handle negative distances
        self.reverse = (target_distance < 0.0)
        target_distance = abs(target_distance)

        # Distance robot takes to accelerate from rest to max speed
        full_acceleration_distance = 0.5 * acceleration_time * max_speed
        self.acceleration = max_speed / acceleration_time
        # Distance robot takes to decelerate from max speed to rest
        full_deceleration_distance = 0.5 * deceleration_time * max_speed
        self.deceleration = -max_speed / deceleration_time

        # Total distance needed to reach max speed from rest,
        #  and then return back to rest
        distance_for_max_speed = (
            full_acceleration_distance + full_deceleration_distance)

        # If the target distance is greater than the distance it takes
        #  for the robot to accelerate to max speed and back to rest
        #  the motion profile will be trapezoidal
        if target_distance >= distance_for_max_speed:
            # How long the robot should spend at full speed
            full_speed_time = (
                target_distance - distance_for_max_speed) / max_speed
            # Time at which robot should transition from acceleration
            #  to constant max speed
            self.acceleration_end_time = acceleration_time
            # Time when robot should start decelerating to rest
            self.deceleration_start_time = self.acceleration_end_time + \
                full_speed_time
            # The time at which the profile will be completed
            self.end_time = self.deceleration_start_time + deceleration_time
            self.max_speed = max_speed

        # If it can't accelerate all the way to max speed and back
        #  without overshooting, the motion profile will be triangular
        else:
            # Distance to get to the top speed the robot will reach - since
            #  it doesn't have time to accelerate fully this is not the
            #  same as full_acceleration_distance
            # This is calculated using the fact that the acceleration and
            #  deceleration distances for a triangular motion profile are
            #  directly proportional to the ratio of the acceleration and
            #  deceleration times
            partial_acceleration_distance = target_distance * \
                (acceleration_time / (acceleration_time + deceleration_time))
            # Time to get to the top speed the robot will actually reach
            # x = 0.5at^2 -> t = sqrt(2x/a)
            self.acceleration_end_time = math.sqrt(
                (2 * partial_acceleration_distance) / self.acceleration)
            # Robot should start decelerating the moment it reaches max speed
            self.deceleration_start_time = self.acceleration_end_time
            # Maximum speed the robot will be able to reach
            #  before it needs to begin deceleration
            self.max_speed = self.acceleration_end_time * self.acceleration
            # Time it will take the robot to declerate from
            #  the maximum speed it will reach back to rest - how much time
            #  will the robot need to stop
            deceleration_time = -self.max_speed / self.deceleration
            # Time at which the motion profile should be completed
            self.end_time = self.acceleration_end_time + deceleration_time

    # position() can be used for times after the end of the profile,
    #  this is so that if the PID hasn't got the robot to the
    #  target position by the end it can keep reducing the position
    #  error till it reaches the target position
    def position(self, time):
        """Get the optimal position at a specific time"""

        # Inner helper function to find the distance traveled
        #  when accelerating from an initial velocity for a time period
        def distance(initial_speed, acceleration, time):
            """Find the distance traveled when accelerating from
             `initial_speed` at `acceleration` for `time`
            """
            # delta_x = vi + 1/2(at^2)
            return (initial_speed * time) + (0.5 * acceleration * time * time)

        position = 0
        # Acceleration phase - time is clamped between zero and
        #  acceleration time
        acceleration_phase_time = max(0, min(time, self.acceleration_end_time))
        position += distance(0.0, self.acceleration, acceleration_phase_time)

        # Max speed phase - time is clamped between 0 and time change between
        #  acceleration and deceleration
        max_speed_phase_time = max(0,
                                   min(time, self.deceleration_start_time) -
                                   self.acceleration_end_time)
        position += max_speed_phase_time * self.max_speed

        # Deceleration phase - time is clamped between zero and the time
        #  change between deceleration and the end of the motion profile
        deceleration_phase_time = min(
            max(0, time - self.deceleration_start_time),
            self.end_time - self.deceleration_start_time)
        position += distance(self.max_speed, self.deceleration,
                             deceleration_phase_time)

        # Handle reverse (negative) directions
        return position if not self.reverse else -position


class ProfileExecutor:
    def __init__(
            self, pid_coefs: PIDCoefficients, motion_profile: MotionProfile,
            input_source: Callable[[], float], output: Callable[[float], None],
            acceptable_error_margin: float):
        """Wrapper for a PID controller and a motion profile. Ties
         them together for seemless profile execution

        Uses `input_source` to retrieve current input for motion profile,
         and `output` to write PID output. `acceptable_error_margin` is the
         acceptable amount of error as a decimal.
        """

        self.pid = PIDController(pid_coefs, 1.0, -1.0)
        self.profile_start_time = wpilib.Timer.getFPGATimestamp()
        self.motion_profile = motion_profile
        self.input_source = input_source
        self.output = output
        self.acceptable_error_margin = acceptable_error_margin

    def update(self) -> bool:
        """Updates motion profile and writes output. Returns `True`
         if profile is completed (robot is within error margin of
         target), otherwise `False`.
        """
        time_delta = wpilib.Timer.getFPGATimestamp() - self.profile_start_time

        current_goal_position = self.motion_profile.position(time_delta)

        current_input = self.input_source()

        output = self.pid.get_output(current_input, current_goal_position)
        self.output(output)
        final_position = self.motion_profile.position(
            self.motion_profile.end_time)

        error = abs(final_position - current_input) / \
            abs(final_position)
        return error < self.acceptable_error_margin
