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

        self.speed = 0
        self.last_speed = 0
        self.safety_enabled = True

        # Arm encoder
        # self.encoder = wpilib.Encoder(*ArmConstants.ENCODER_PORTS)

        # Upper arm limit switch. Trips when the arm has reached the maximum height mechanically possible.
        self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        # self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

        # Put bool to the dashboard for arm locking.
        # Used in commands.arm.lock_arm.py
        wpilib.SmartDashboard.putBoolean("Arm Locked?", False)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putBoolean("Upper Limit Pressed?", self.upper_limit_pressed())

        # Gradually decrease speed to zero to avoid the arm slamming down
        if self.speed == 0:
            self.speed = max(self.last_speed - 0.01, 0)

        # Stop the motor if we try to exceed the limits
        if not self._safe_to_drive(self.speed):
            self.speed = 0

        # Drive the motor at a desired speed.
        # A certain amount of power is needed to keep the arm raised.
        self.arm_motors.set(self.speed)

        # Set a variable for the next loop
        self.last_speed = self.speed

        self._update_dashboard()

    def _update_dashboard(self):
        wpilib.SmartDashboard.putNumber("Arm Speed", self.get_speed())
        wpilib.SmartDashboard.putBoolean("Safety Enabled?", self.is_safety_enabled())

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

    def set_speed(self, speed: float) -> None:
        self.speed = speed

    def get_speed(self) -> float:
        return self.speed

    def upper_limit_pressed(self) -> bool:
        return self.upper_limit.get()

    def set_safety(self, enabled: bool) -> None:
        self.safety_enabled = enabled

    def is_safety_enabled(self) -> bool:
        return self.safety_enabled
