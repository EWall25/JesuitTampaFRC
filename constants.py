from math import pi


class DriveConstants:
    FRONT_RIGHT_MOTOR_PORT = 0
    BACK_RIGHT_MOTOR_PORT = 1
    FRONT_LEFT_MOTOR_PORT = 2
    BACK_LEFT_MOTOR_PORT = 3

    LEFT_MOTORS_INVERTED = False
    RIGHT_MOTORS_INVERTED = False

    TELEOP_DEFAULT_DRIVE_SPEED = 0.5
    TELEOP_BOOST_DRIVE_SPEED = 1
    TELEOP_TURN_SPEED = 0.5

    # How many arbitrary units the robot will accelerate in a second
    # For example, a value of 2 will make the robot accelerate to full power in 0.5 seconds
    TELEOP_DRIVE_ACCELERATION_PER_SECOND = 2

    WHEELBASE_METRES = 21.8625 / 39.37

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

    DRIVE_TOLERANCE_METRES = 0.12
    TURN_TOLERANCE_DEGREES = 3

    LEFT_ENCODER_PORT = 2
    RIGHT_ENCODER_PORT = 3
    # LEFT_ENCODER_PORTS = (0, 1)
    # RIGHT_ENCODER_PORTS = (2, 3)

    ENCODER_CPR = 360
    WHEEL_DIAMETER_METRES = 0.1524
    ENCODER_DISTANCE_PER_PULSE = WHEEL_DIAMETER_METRES * pi / ENCODER_CPR

    IMU_PORT = 1


class ArmConstants:
    MOTOR_1_PORT = 4
    MOTOR_2_PORT = 5

    ENCODER_PORTS = (2, 3)

    UPPER_LIMIT_SWITCH_PORT = 6
    LOWER_LIMIT_SWITCH_PORT = 7

    LOWER_HUB_HEIGHT_DEGREES = 1
    RAMP_HEIGHT_DEGREES = 1

    ROTATION_P = 1
    ROTATION_I = 0
    ROTATION_D = 0

    ROTATION_TOLERANCE_DEGREES = 3


class DriverStationConstants:
    DRIVER_CONTROLLER_PORT = 0

    DRIVE_STICK = 1
    TURN_STICK = 4


class AutoConstants:
    pass
