import typing

import commands2

from constants import ArmConstants
from subsystems.arm_subsystem import ArmSubsystem


class DirectArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers.
        :param arm: The arm subsystem
        :param movement: A function returning the robot arm's intended speed.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement

        self.addRequirements([arm])

    def initialize(self) -> None:
        print("lord help us")

    def execute(self) -> None:
        self.arm.set_height(self.movement())
