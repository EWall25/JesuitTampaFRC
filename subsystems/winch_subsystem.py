import commands2
import wpilib

from constants import WinchConstants


class WinchSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Winch motor
        self.motor = wpilib.Spark(WinchConstants.WINCH_MOTOR_PORT)

    def drive(self, power: float = WinchConstants.WINCH_DRIVE_POWER) -> None:
        """
        Pull/release the winch chord.
        :param power: The power to drive the motor at
        """

        self.motor.set(power)

    def stop(self) -> None:
        """
        Stops the winch motor.
        """

        self.motor.stopMotor()
