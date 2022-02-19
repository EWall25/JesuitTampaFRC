import commands2
import wpilib

from constants import ArmConstants


class ArmSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Arm motor
        self.motor_1 = wpilib.Spark(ArmConstants.MOTOR_1_PORT)
        self.motor_2 = wpilib.Spark(ArmConstants.MOTOR_2_PORT)
        self.motor = wpilib.MotorControllerGroup(self.motor_1, self.motor_2)

        # Arm encoder
        self.encoder = wpilib.Encoder(*ArmConstants.ENCODER_PORTS)

        # Upper arm limit switch. Trips when the arm has reached the maximum height mechanically possible.
        self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

        self._safety = True

    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        builder.setSmartDashboardType("Arm")
        builder.addBooleanProperty("Safety Enabled", self.get_safety, self.set_safety)
        builder.addBooleanProperty("Upper Limit", self.at_upper_limit, lambda *args: None)
        builder.addBooleanProperty("Lower Limit", self.at_lower_limit, lambda *args: None)

    def _safe_to_move(self, value: float) -> bool:
        """
        Is it safe to drive the motor?
        :param value: The power/voltage being fed to the motor. Positive should be clockwise.
        """

        # Assume it's safe to move the arm if safety is disabled
        if not self._safety:
            return True

        # Check if the arm is hitting its limits
        if (value > 0 and self.at_upper_limit()) or (value < 0 and self.at_lower_limit()):
            # Stop the motors and tell the code it's not safe to move
            self.motor.stopMotor()
            return False

        # It's safe to move the arm
        return True

    def move(self, value: float) -> None:
        if self._safe_to_move(value):
            self.motor.set(value)

    def set_voltage(self, volts: float) -> None:
        if self._safe_to_move(volts):
            self.motor.setVoltage(volts)

    def stop(self) -> None:
        self.motor.stopMotor()

    def set_safety(self, enabled: bool):
        """
        Enables or disables limits.
        :param enabled: Should limits be enabled?
        """

        self._safety = enabled

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
        """
        Gets the arm's rotation.
        :return: The arm motor's rotation in degrees
        """

        return self.encoder.getDistance()

    def get_speed(self) -> float:
        """
        Gets the arm's speed.
        :return: The arm motor's speed in degrees per second
        """

        return self.encoder.getRate()

    def get_safety(self) -> bool:
        """
        Gets if the limits are enabled.
        :return: Are limits enabled?
        """

        return self._safety
