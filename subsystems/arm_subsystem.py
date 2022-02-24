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
        # self.upper_limit = wpilib.DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT)

        # Lower arm limit switch. Trips when the arm has reached the minimum height mechanically possible.
        # self.lower_limit = wpilib.DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT)

    '''
    def periodic(self) -> None:
        # Put values to the Smart Dashboard
        self._update_dashboard()

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
        self.arm_motors.set(value)

    def set_voltage(self, volts: float) -> None:
        self.arm_motors.setVoltage(volts)

    def stop(self) -> None:
        self.arm_motors.stopMotor()

    def at_upper_limit(self) -> bool:
        """
        Can the arm move further upwards?
        :return: Has the upper limit switch been tripped?
        """

        # return self.upper_limit.get()
        raise NotImplementedError

    def at_lower_limit(self) -> bool:
        """
        Can the arm move further downwards?
        :return: Has the lower limit switch been tripped?
        """

        # return self.lower_limit.get()
        raise NotImplementedError

    def get_position(self) -> float:
        """
        Gets the arm's rotation.
        :return: The arm motor's rotation in degrees
        """

        # return self.encoder.getDistance()
        raise NotImplementedError

    def get_speed(self) -> float:
        """
        Gets the arm's speed.
        :return: The arm motor's speed in degrees per second
        """

        # return self.encoder.getRate()
        raise NotImplementedError
