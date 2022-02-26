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

        # Arm encoder
        # self.encoder = wpilib.Encoder(*ArmConstants.ENCODER_PORTS)

        # Upper arm limit switch. Trips when the arm has reached the maximum height mechanically possible.
        # self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        # self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

    def periodic(self) -> None:
        # Drive the motor to keep the arm at a specified height
        self.arm_motors.set(self.speed)

        wpilib.SmartDashboard.putNumber("Speed/Height", self.get_height())

    '''
    def _update_dashboard(self):
        wpilib.SmartDashboard.putBoolean("Safety Enabled", self.get_safety())
        wpilib.SmartDashboard.putBoolean("Upper Limit Pressed", self.at_upper_limit())
        wpilib.SmartDashboard.putBoolean("Lower Limit Pressed", self.at_lower_limit())
    
    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        builder.setSmartDashboardType("Arm")
        builder.addBooleanProperty("Safety Enabled", self.get_safety, self.set_safety)
        builder.addBooleanProperty("Upper Limit", self.at_upper_limit, lambda *args: None)
        builder.addBooleanProperty("Lower Limit", self.at_lower_limit, lambda *args: None)
    '''

    def move(self, value: float) -> None:
        self.speed += value
        self.speed = util.clamp(self.speed, 0, 1)

    def set_height(self, speed: float) -> None:
        self.speed = speed

    def get_height(self) -> float:
        return self.speed

    def drop(self) -> None:
        self.speed = 0
