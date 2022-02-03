import math

import commands2
import wpilib
import wpilib.drive
from ctre import PigeonIMU, CANCoder, SensorTimeBase

from constants import *


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        # Front-left drive motor
        self.fl_motor = wpilib.Spark(FRONT_LEFT_MOTOR_PORT)
        # Back-left drive motor
        self.bl_motor = wpilib.Spark(BACK_LEFT_MOTOR_PORT)
        # Front-right drive motor
        self.fr_motor = wpilib.Spark(FRONT_RIGHT_MOTOR_PORT)
        # Back-right drive motor
        self.br_motor = wpilib.Spark(BACK_RIGHT_MOTOR_PORT)

        # Both drive motors on the left side of the robot
        self.l_motors = wpilib.MotorControllerGroup(self.fl_motor, self.bl_motor)
        self.l_motors.setInverted(False)

        # Both drive motors on the right side of the robot
        self.r_motors = wpilib.MotorControllerGroup(self.fr_motor, self.br_motor)
        self.r_motors.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motors, self.r_motors)

        # Left side encoder
        # self.l_encoder = CANCoder(2)
        self.l_encoder = wpilib.Encoder(0, 1)
        self.l_encoder.setReverseDirection(False)

        # Right side encoder
        #self.r_encoder = CANCoder(3)
        self.r_encoder = wpilib.Encoder(2, 3)
        self.r_encoder.setReverseDirection(True)


        # Configure the encoders to return a value in inches
        # Uses the wheel diameter and number of "counts" per motor rotation to calculate distance
        #self.l_encoder.configFeedbackCoefficient(6 * math.pi / 4096, "inches", SensorTimeBase.PerSecond)
        #self.r_encoder.configFeedbackCoefficient(6 * math.pi / 4096, "inches", SensorTimeBase.PerSecond)
        self.l_encoder.setDistancePerPulse(6 * math.pi / 360)
        self.r_encoder.setDistancePerPulse(6 * math.pi / 360)

        # Put encoders to Smart Dashboard
        wpilib.SmartDashboard.putData("Left Encoder", self.l_encoder)
        wpilib.SmartDashboard.putData("Right Encoder", self.r_encoder)

        # Inertia Measurement Unit/Gyroscope
        self.imu = PigeonIMU(1)

    def periodic(self) -> None:
        # Update the robot's heading direction on the Smart Dashboard
        wpilib.SmartDashboard.putNumber("Heading", self.get_heading())
        print(self.l_encoder.get())

    def arcade_drive(self, forward: float, rotation: float) -> None:
        """
        Drives the robot using arcade controls.

        :param forward: Forward movement rate
        :param rotation: Rotation rate
        """

        self.drive.arcadeDrive(forward, rotation, False)

    def stop(self) -> None:
        """
        Stops the robot's motors.
        """

        self.drive.stopMotor()

    def reset_heading(self) -> None:
        """
        Zeroes the gyroscope's heading.
        """

        self.imu.setYaw(0)

    def get_heading(self) -> float:
        """
        Gets the robot's heading direction.
        :return: The robot's heading direction in degrees from -180 to 180
        """

        # getYawPitchRoll returns an error code and an array containing the yaw, pitch and roll
        # Get that array then return the yaw
        ypr = self.imu.getYawPitchRoll()[1]
        return ypr[0]

    def get_average_distance(self) -> float:
        """
        Gets the distance travelled by the robot.
        :return: The average travelled distance by both drive encoders.
        """

        # Add together the left encoder and right encoder's position then average them by dividing by 2
        # return (self.l_encoder.getPosition() + self.r_encoder.getPosition()) / 2
        return (self.l_encoder.getDistance() + self.r_encoder.getDistance()) / 2

    def reset_encoders(self) -> None:
        """
        Resets the encoders' values to 0
        """

        # self.l_encoder.setPosition(0)
        # self.r_encoder.setPosition(0)
        self.l_encoder.reset()
        self.r_encoder.reset()
