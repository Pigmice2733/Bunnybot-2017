from pyfrc.physics.drivetrains import two_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller
        self.controller.add_device_gyro_channel('adxrs450_spi_0_angle')
        self.encoder_position = 0.0

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        front_left = -hal_data['CAN'][1]['value']
        front_right = hal_data['CAN'][3]['value']

        # CAN outputs to the hal are multiplied by the duty cycle, 1023
        talon_duty_cycle = 1023
        # Quadrature encoders have 4 edges per revolution
        edges_per_revolution = 4
        position_change = (front_left / talon_duty_cycle) * tm_diff
        self.encoder_position += position_change * edges_per_revolution

        # Scale encoder value for sim
        hal_data['CAN'][1]['enc_position'] = 9.98 * self.encoder_position

        rotation, speed = two_motor_drivetrain(front_left, front_right, 3,
                                               0.025)

        self.controller.drive(speed, rotation, tm_diff)
