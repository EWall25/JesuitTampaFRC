import commands2
import wpilib

from constants import ArmConstants


class ArmSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        # Arm motor
        self.left_motor = wpilib.Spark(ArmConstants.LEFT_MOTOR_PORT)
        self.right_motor = wpilib.Spark(ArmConstants.RIGHT_MOTOR_PORT)
        self.arm_motors = wpilib.MotorControllerGroup(self.left_motor, self.right_motor)

        # Arm encoder
        # self.encoder = wpilib.Encoder(*ArmConstants.ENCODER_PORTS)

        # Upper arm limit switch. Trips when the arm has reached the maximum height mechanically possible.
        self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        # self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

        self._limits_enabled = True

    def periodic(self) -> None:
        self._update_dashboard()

    def _update_dashboard(self):
        wpilib.SmartDashboard.putNumber("Arm Speed", self.get_power())
        wpilib.SmartDashboard.putBoolean("Upper Limit Tripped?", self.upper_limit_pressed())

    def set_power(self, goal: float) -> None:
        # If the limits are enabled then prevent us from exceeding the power needed to go past the limit
        # TODO: Find this power
        power = min(0.8, goal) if self.limits_enabled() else goal

        # Drive the arm motors
        self.arm_motors.set(power)

    def get_power(self) -> float:
        return self.arm_motors.get()

    def upper_limit_pressed(self) -> bool:
        return self.upper_limit.get()

    def set_limits(self, enabled: bool) -> None:
        self._limits_enabled = enabled

    def limits_enabled(self) -> bool:
        return self._limits_enabled
