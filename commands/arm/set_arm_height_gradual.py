import commands2
import wpimath.filter

from subsystems.arm_subsystem import ArmSubsystem


class SetArmHeight(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem, height: float):
        """
        Gradually moves the arm to a specified height.
        :param arm: The arm subsystem
        :param height: The height to move the arm to as a PWM value. Ranges 0-1.
        """

        super().__init__()

        self.arm = arm
        self.target = height
        self.filter = wpimath.filter.SlewRateLimiter(0.2)

        self.addRequirements([arm])

    def initialize(self) -> None:
        self.filter.reset(self.arm.get_speed())

    def execute(self) -> None:
        movement = self.filter.calculate(self.target)
        self.arm.set_speed(movement)

    def isFinished(self) -> bool:
        return self.arm.get_speed() == self.target
