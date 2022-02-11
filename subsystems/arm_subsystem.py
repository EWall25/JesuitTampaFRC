import commands2
import wpilib

from constants import ArmConstants


class ArmSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Arm motor
        self.motor = wpilib.Spark(ArmConstants.MOTOR_PORT)

        # Arm encoder
        self.encoder = wpilib.Encoder(*ArmConstants.ENCODER_PORTS)

        # Upper arm limit switch. Trips when the arm has reached the maximum height mechanically possible.
        self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

        self._safety = True

    def move(self, value: float) -> None:
        if self.safety_enabled():
            # Don't move if we're at the arms upper limit
            if value > 0 and self.at_upper_limit():
                return

            # Don't move if we're at the arms lower limit
            if value < 0 and self.at_lower_limit():
                return

        self.motor.set(value)

    def set_voltage(self, volts: float) -> None:
        if self.safety_enabled():
            # Don't move if we're at the arms upper limit
            if volts > 0 and self.at_upper_limit():
                return

            # Don't move if we're at the arms lower limit
            if volts < 0 and self.at_lower_limit():
                return

        self.motor.setVoltage(volts)

    def set_safety(self, safety: bool) -> None:
        """
        Changes whether the arm can move past the limit switches. Safety should be on during normal operation.
        :param safety: Should safety features be on or off
        """

        self._safety = safety

    def at_upper_limit(self) -> bool:
        """
        Can the arm move further upwards?
        :return: Has the upper limit switch been tripped?
        """
        return self.upper_limit.get()

    def at_lower_limit(self) -> bool:
        """
        Can the arm move further downwards?
        :return: Has the lower limit switch been tripped?
        """
        return self.lower_limit.get()

    def get_position(self) -> float:
        return self.encoder.getDistance()

    def get_speed(self) -> float:
        return self.encoder.getRate()

    def safety_enabled(self) -> bool:
        """
        Can the arm move past its limit switches? Should be False during normal operation.
        :return: Are safety features on
        """

        return self._safety
