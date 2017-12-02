"""Test module for motion_profiles.py"""

import pytest

from common.motion_profiles import MotionProfile


class TestMotionProfile:
    """Test class for MotionProfile"""

    def create_test_profile(acceleration, deceleration, max_speed,
                            acceleration_time, deceleration_time, end_time,
                            reverse):
        profile = MotionProfile(1, 1, 1, 5)
        profile.acceleration = acceleration
        profile.acceleration_end_time = acceleration_time
        profile.deceleration = deceleration
        profile.deceleration_start_time = deceleration_time
        profile.max_speed = max_speed
        profile.end_time = end_time
        profile.reverse = reverse
        return profile

    def approx_equal(expected, actual):
        assert expected.acceleration == pytest.approx(actual.acceleration)
        assert expected.deceleration == pytest.approx(actual.deceleration)
        assert expected.acceleration_end_time == pytest.approx(
            actual.acceleration_end_time)
        assert expected.deceleration_start_time == pytest.approx(
            actual.deceleration_start_time)
        assert expected.max_speed == pytest.approx(actual.max_speed)
        assert expected.end_time == pytest.approx(actual.end_time)
        assert expected.reverse == pytest.approx(actual.reverse)

    def test_generate_motion_profile_reach_max_speed(self):
        """Test MotionProfile.generate_motion_profile() with robot
         reaching max speed - trapezoidal"""
        correct_motion_profile = TestMotionProfile.create_test_profile(
            2, -3, 6, 3, 4.5, 6.5, False)
        generated_motion_profile = MotionProfile(3, 2, 6, 24)

        TestMotionProfile.approx_equal(correct_motion_profile,
                                       generated_motion_profile)

    def test_generate_motion_profile_no_max_speed(self):
        """Test MotionProfile.generate_motion_profile() where
         the robot should never reach reach max speed - triangular"""
        correct_motion_profile = TestMotionProfile.create_test_profile(
            1.5, -1.2, 4.5, 3, 3, 6.75, False)
        generated_motion_profile = MotionProfile(4, 5, 6, 15.1875)

        TestMotionProfile.approx_equal(correct_motion_profile,
                                       generated_motion_profile)

        correct_motion_profile = TestMotionProfile.create_test_profile(
            2, -2.15, 8.6, 4.3, 4.3, 8.3, False)
        generated_motion_profile = MotionProfile(4.5, 9 / (2.15), 9, 35.69)

        TestMotionProfile.approx_equal(correct_motion_profile,
                                       generated_motion_profile)

    def test_position_calculations_trapezoid(self):
        """Test MotionProfile position() for a trapezoidal profile"""
        motion_profile = TestMotionProfile.create_test_profile(
            2, -3, 6, 3, 4.5, 6.5, False)
        # Test 4 points on trapezoid - 1 during acceleration,
        #  1 during max-speed, and 2 during deceleration
        times = [2, 4, 5, 6]
        correct_positions = [4, 15, 20.625, 23.625]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

    def test_position_calculations_triangle(self):
        """Test MotionProfile position() for a triangular profile"""
        motion_profile = TestMotionProfile.create_test_profile(
            1.5, -1.2, 4.5, 3, 3, 6.75, False)
        # Test 4 points on triangle - 2 during acceleration,
        #  2 during deceleration
        times = [1, 2, 4, 5.75]
        correct_positions = [0.75, 3, 10.65, 14.5875]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

    def test_negative_distance(self):
        """Test that MotionProfile handles negative distances correctly"""
        motion_profile = TestMotionProfile.create_test_profile(
            1.5, -1.2, 4.5, 3, 3, 6.75, True)
        # Test 4 points on triangle - 2 during acceleration,
        #  2 during deceleration
        times = [1, 2, 4, 5.75]
        correct_positions = [-0.75, -3, -10.65, -14.5875]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)

        motion_profile = TestMotionProfile.create_test_profile(
            2, -3, 6, 3, 4.5, 6.5, True)
        # Test 4 points on trapezoid - 1 during acceleration,
        #  1 during max-speed, and 2 during deceleration
        times = [2, 4, 5, 6]
        correct_positions = [-4, -15, -20.625, -23.625]

        calculated_positions = list(
            map(lambda time: motion_profile.position(time), times))

        assert correct_positions == pytest.approx(calculated_positions)
