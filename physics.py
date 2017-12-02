from pyfrc.physics.drivetrains import two_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller
        self.controller.add_device_gyro_channel('adxrs450_spi_0_angle')
        self.encoder_position = 0.0

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        front_left = hal_data['CAN'][1]['value']
        front_right = -hal_data['CAN'][3]['value']

        self.encoder_position += front_left * tm_diff

        hal_data['CAN'][1]['enc_position'] += hal_data['CAN'][1]['value'] / \
            1023 * tm_diff * 5000

        rotation, speed = two_motor_drivetrain(front_left, front_right, 3,
                                               0.025)

        self.controller.drive(speed, rotation, tm_diff)
