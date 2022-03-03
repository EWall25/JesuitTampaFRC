import typing

import commands2
import wpilib
import wpimath.filter

from constants import ArmConstants
from subsystems.arm_subsystem import ArmSubsystem


class PowerControlArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers, directly setting the arm's speed.
        :param arm: The arm subsystem
        :param movement: A function returning the robot arm's intended speed.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement

        # self.filter = wpimath.filter.SlewRateLimiter(ArmConstants.ARM_RATE_LIMIT_PER_SECOND)

        self.addRequirements([arm])

    def execute(self) -> None:
        # Set the speed of the arm motor
        # movement = self.filter.calculate(self.movement())
        movement = self.movement()
        self.arm.set_speed(movement)
