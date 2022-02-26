from math import pi


class DriveConstants:
    FRONT_RIGHT_MOTOR_PORT = 0
    BACK_RIGHT_MOTOR_PORT = 1
    FRONT_LEFT_MOTOR_PORT = 2
    BACK_LEFT_MOTOR_PORT = 3

    LEFT_MOTORS_INVERTED = False
    RIGHT_MOTORS_INVERTED = True

    TELEOP_DEFAULT_DRIVE_SPEED = 0.5
    TELEOP_BOOST_DRIVE_SPEED = 1
    TELEOP_TURN_SPEED = 0.5

    # How many arbitrary units the robot will accelerate in a second
    # For example, a value of 2 will make the robot accelerate to full power in 0.5 seconds
    TELEOP_DRIVE_ACCELERATION_PER_SECOND = 2

    DRIVE_TOLERANCE_METRES = 0.12
    TURN_TOLERANCE_DEGREES = 3

    LEFT_ENCODER_PORT = 1   # TODO: Find correct encoder ports
    RIGHT_ENCODER_PORT = 2
    # LEFT_ENCODER_PORTS = (0, 1)
    # RIGHT_ENCODER_PORTS = (2, 3)

    ENCODER_CPR = 4096
    WHEEL_DIAMETER_METRES = 0.1524
    ENCODER_DISTANCE_PER_PULSE = WHEEL_DIAMETER_METRES * pi / ENCODER_CPR

    IMU_PORT = 1

    WHEELBASE_METRES = 21.8625 / 39.37

    '''
    S_VOLTS = 1
    V_VOLT_SECONDS_PER_METRE = 0.5
    A_VOLT_SECONDS_SQUARED_PER_METRE = 0

    TURN_DEGREES_P = 1
    TURN_DEGREES_I = 0
    TURN_DEGREES_D = 0

    VELOCITY_P = 1
    VELOCITY_I = 0
    VELOCITY_D = 0

    DRIVE_METRES_P = 1
    DRIVE_METRES_I = 0
    DRIVE_METRES_D = 0
    '''


class ArmConstants:
    LEFT_MOTOR_PORT = 4
    RIGHT_MOTOR_PORT = 5

    # ENCODER_PORTS = (2, 3)

    # UPPER_LIMIT_SWITCH_PORT = 6
    # LOWER_LIMIT_SWITCH_PORT = 7

    ARM_SPEED = 0.01

    LOWER_HUB_HEIGHT_PWM = 0.6
    RAMP_HEIGHT_PWM = 0.65


class WinchConstants:
    WINCH_MOTOR_PORT = 6

    WINCH_DRIVE_TIME_SECONDS = 1
    WINCH_DRIVE_POWER = 0.3


class DriverStationConstants:
    DRIVER_CONTROLLER_PORT = 0
    ARM_CONTROLLER_PORT = 1

    DRIVE_STICK = 1             # Left Stick
    TURN_STICK = 4              # Right Stick
    SPEED_TOGGLE_BUTTON = 6     # Right Bumper

    ARM_AXIS = 1                # Left Stick
    WINCH_BUTTON = 4            # Y Button
    RAMP_BUTTON = 3             # X Button
    LOWER_HUB_BUTTON = 1        # A Button
    ARM_MODE_BUTTON = 8         # Start Button


class AutoConstants:
    DRIVE_AWAY_FROM_HUB_DISTANCE_FEET = -7.5
    DRIVE_AWAY_FROM_HUB_SPEED = 0.5
