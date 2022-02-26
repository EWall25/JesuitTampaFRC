import typing

import commands2

from subsystems.arm_subsystem import ArmSubsystem


class DefaultArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers.
        :param arm: The arm subsystem
        :param movement: A function returning how much should be added to the arm's height.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement

        self.addRequirements([arm])

    def execute(self) -> None:
        self.arm.move(self.movement())
