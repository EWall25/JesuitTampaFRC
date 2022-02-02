import commands2
import ctre
import wpilib
import wpilib.drive
from ctre import PigeonIMU

from constants import *


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.fl_motor = wpilib.Spark(FRONT_LEFT_MOTOR_PORT)
        self.bl_motor = wpilib.Spark(BACK_LEFT_MOTOR_PORT)
        self.fr_motor = wpilib.Spark(FRONT_RIGHT_MOTOR_PORT)
        self.br_motor = wpilib.Spark(BACK_RIGHT_MOTOR_PORT)

        # Encoder on CAN port 2
        self.l_encoder = ctre.CANCoder(1)

        # Encoder on CAN port 1
        self.r_encoder = ctre.CANCoder(2)

        self.drive = wpilib.drive.DifferentialDrive(
            wpilib.SpeedControllerGroup(self.fl_motor, self.bl_motor),
            wpilib.SpeedControllerGroup(self.fr_motor, self.br_motor)
        )

        # Inertia Measurement Unit on CAN port 0
        self.imu = PigeonIMU(0)

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

        return self.imu.getYawPitchRoll()[0]

    def get_average_distance(self) -> float:
        """
        Gets the distance travelled by the robot.
        :return: The average travelled distance by both drive encoders.
        """

        # Add together the left encoder and right encoder's position then average them by dividing by 2
        return (self.l_encoder.getPosition() + self.r_encoder.getPosition()) / 2

    def reset_encoders(self) -> None:
        """
        Resets the encoders' values to 0
        """

        self.l_encoder.setPosition(0)
        self.r_encoder.setPosition(0)
