import commands2
import wpilib

from constants import WinchConstants


class WinchSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        self.enabled = True

        # Winch motor
        self.motor = wpilib.Spark(WinchConstants.WINCH_MOTOR_PORT)
        self.motor.setInverted(WinchConstants.MOTOR_INVERTED)

    def periodic(self) -> None:
        self._update_dashboard()

    def _update_dashboard(self):
        wpilib.SmartDashboard.putBoolean("Winch Operable?", self.is_enabled())

    def drive(self, power: float = WinchConstants.WINCH_DRIVE_POWER) -> None:
        """
        Pull/release the winch chord.
        :param power: The power to drive the motor at
        """
        if self.enabled:
            self.motor.set(power)

    def stop(self) -> None:
        """
        Stops the winch motor.
        """

        self.motor.stopMotor()

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def is_enabled(self) -> bool:
        return self.enabled
