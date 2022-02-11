import commands2
import wpilib
import wpilib.drive
import wpimath.kinematics
from ctre import PigeonIMU, CANCoder, SensorTimeBase
from wpimath.geometry import Pose2d, Rotation2d

from constants import DriveConstants


class DriveSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Front-left drive motor
        self.fl_motor = wpilib.Spark(DriveConstants.FRONT_LEFT_MOTOR_PORT)
        # Back-left drive motor
        self.bl_motor = wpilib.Spark(DriveConstants.BACK_LEFT_MOTOR_PORT)
        # Front-right drive motor
        self.fr_motor = wpilib.Spark(DriveConstants.FRONT_RIGHT_MOTOR_PORT)
        # Back-right drive motor
        self.br_motor = wpilib.Spark(DriveConstants.BACK_RIGHT_MOTOR_PORT)

        # Both drive motors on the left side of the robot
        self.l_motors = wpilib.MotorControllerGroup(self.fl_motor, self.bl_motor)
        self.l_motors.setInverted(False)

        # Both drive motors on the right side of the robot
        self.r_motors = wpilib.MotorControllerGroup(self.fr_motor, self.br_motor)
        self.r_motors.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motors, self.r_motors)

        # Left side encoder
        # self.l_encoder = CANCoder(2)
        self.l_encoder = wpilib.Encoder(*DriveConstants.LEFT_ENCODER_PORTS)
        self.l_encoder.setReverseDirection(False)

        # Right side encoder
        # self.r_encoder = CANCoder(3)
        self.r_encoder = wpilib.Encoder(*DriveConstants.RIGHT_ENCODER_PORTS)
        self.r_encoder.setReverseDirection(True)

        # Configure the encoders to return a value in metres
        # Uses the wheel diameter and number of "counts" per motor rotation to calculate distance
        # self.l_encoder.configFeedbackCoefficient(6 * math.pi / 4096, "inches", SensorTimeBase.PerSecond)
        # self.r_encoder.configFeedbackCoefficient(6 * math.pi / 4096, "inches", SensorTimeBase.PerSecond)
        self.l_encoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE)
        self.r_encoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE)

        # Put encoders to Smart Dashboard
        wpilib.SmartDashboard.putData("Left Encoder", self.l_encoder)
        wpilib.SmartDashboard.putData("Right Encoder", self.r_encoder)

        # Inertia Measurement Unit/Gyroscope
        self.imu = PigeonIMU(DriveConstants.IMU_PORT)

        # The robot's pose (position and rotation on the field).
        # This will be updated periodically.
        self.pose = Pose2d()

        # Encoders MUST be reset to 0 before instantiating odometry
        self.reset_encoders()

        # Create the odometry object.
        # Odometry allows us to know the position of the robot during autonomous.
        # It shouldn't be used during teleop because of drift from colliding with
        # other robots.
        # See periodic() for why the heading needs to be negated. I'm not writing it again.
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(
            Rotation2d.fromDegrees(-self.get_heading()), Pose2d(0, 0, Rotation2d()))

        # Create kinematics. Kinematics allow us to learn the wheel speeds from the robot's speed.
        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(DriveConstants.WHEELBASE_METRES)

    def periodic(self) -> None:
        # Get the gyro angle. We have to negate it because WPILib classes expect
        # a negative value as the robot turns clockwise. Our gyro returns a positive
        # value as the robot turns clockwise.
        heading = Rotation2d.fromDegrees(-self.get_heading())

        # Get the distance the robot has driven in metres.
        l_distance = self.get_left_wheel_distance()
        r_distance = self.get_right_wheel_distance()

        # Update the robot's position and rotation on the field
        self.pose = self.odometry.update(heading, l_distance, r_distance)

        # Update the robot's heading direction on the Smart Dashboard
        wpilib.SmartDashboard.putNumber("Heading", self.get_heading())

    def arcade_drive(self, forward: float, rotation: float) -> None:
        """
        Drives the robot using arcade controls.

        :param forward: Forward movement rate
        :param rotation: Rotation rate
        """

        self.drive.arcadeDrive(forward, rotation, False)

    def tank_drive_volts(self, left: float, right: float) -> None:
        """
        Drives the robot by setting the motor voltages.

        :param left: The voltage to apply to the left motor
        :param right: The voltage to apply to the right motor
        """

        # Apply voltages to motors
        self.l_motors.setVoltage(left)
        self.r_motors.setVoltage(right)

        # Tell the system that we're driving
        self.drive.feed()

    def stop(self) -> None:
        """
        Stops the robot's motors.
        """

        self.drive.stopMotor()

    def get_heading(self) -> float:
        """
        Gets the robot's heading direction.
        :return: The robot's heading direction in degrees from -180 to 180
        """

        # getYawPitchRoll returns an error code and an array containing the yaw, pitch and roll
        # Get that array then return the yaw
        ypr = self.imu.getYawPitchRoll()[1]
        return ypr[0]

    def get_left_wheel_distance(self) -> float:
        """
        Gets the distance travelled by the robot's left wheel.
        :return: The distance travelled by the left wheel in metres.
        """

        return self.l_encoder.getDistance()

    def get_right_wheel_distance(self) -> float:
        """
        Gets the distance travelled by the robot's right wheel.
        :return: The distance travelled by the right wheel in metres.
        """

        return self.r_encoder.getDistance()

    def get_average_distance(self) -> float:
        """
        Gets the distance travelled by the robot.
        :return: The average distance travelled by both drive motors in metres.
        """

        # Add together the left encoder and right encoder's position then average them by dividing by 2
        # return (self.l_encoder.getPosition() + self.r_encoder.getPosition()) / 2
        return (self.get_left_wheel_distance() + self.get_right_wheel_distance()) / 2

    def get_wheel_speeds(self) -> wpimath.kinematics.DifferentialDriveWheelSpeeds:
        """
        Gets both wheels' speeds.
        :return: The wheels' speeds as a DifferentialDriveWheelSpeeds object
        """
        return wpimath.kinematics.DifferentialDriveWheelSpeeds(self.l_encoder.getRate(), self.r_encoder.getRate())

    def get_pose(self) -> Pose2d:
        """
        Gets the robot's position and rotation on the field from the odometry.
        :return: The robot's pose
        """

        return self.pose

    def reset_heading(self) -> None:
        """
        Zeroes the gyroscope's heading.
        """

        self.imu.setYaw(0)

    def reset_encoders(self) -> None:
        """
        Resets the encoders' values to 0
        """

        # self.l_encoder.setPosition(0)
        # self.r_encoder.setPosition(0)
        self.l_encoder.reset()
        self.r_encoder.reset()

    def reset_pose(self, pose: Pose2d) -> None:
        """
        Resets the robot's position to the specified pose. Also resets the encoders to zero,
        as encoders must be zeroed every time odometry is reset.
        :param pose: The pose to reset the odometry to
        """

        # Get the gyro angle. We have to negate it because WPILib classes expect
        # a negative value as the robot turns clockwise. Our gyro returns a positive
        # value as the robot turns clockwise.
        heading = Rotation2d.fromDegrees(-self.get_heading())

        self.reset_encoders()

        self.odometry.resetPosition(pose, heading)
