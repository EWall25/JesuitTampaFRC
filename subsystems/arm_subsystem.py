import commands2
import wpilib

import util
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

    def periodic(self) -> None:
        self._update_dashboard()

    def _update_dashboard(self):
        wpilib.SmartDashboard.putNumber("Arm Speed", self.get_power())
        wpilib.SmartDashboard.putBoolean("Upper Limit Tripped?", self.upper_limit_pressed())

    def _safe_to_drive(self, power: float) -> bool:
        if self.safety_enabled:
            if self.upper_limit_pressed() and power > 0:
                return False
        return True

    '''
    # def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        builder.setSmartDashboardType("Arm")
        builder.addBooleanProperty("Safety Enabled", self.get_safety, self.set_safety)
        builder.addBooleanProperty("Upper Limit", self.at_upper_limit, lambda *args: None)
        builder.addBooleanProperty("Lower Limit", self.at_lower_limit, lambda *args: None)
    '''

    def set_power(self, power: float) -> None:
        self.arm_motors.set(power)

    def get_power(self) -> float:
        return self.arm_motors.get()

    def upper_limit_pressed(self) -> bool:
        return self.upper_limit.get()
