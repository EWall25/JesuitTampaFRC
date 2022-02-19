import typing

import commands2

from subsystems.arm_subsystem import ArmSubsystem


class DefaultArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            up: typing.Callable[[], float],
            down: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers.
        :param arm: The arm subsystem
        :param up: A function returning the intended upward speed. Ranges 0 to 1.
        :param down: A function returning the intended downward speed. Ranges 0 to 1.
        """

        super().__init__()

        self.arm = arm
        self.up = up
        self.down = down

        self.addRequirements([arm])

    def execute(self) -> None:
        self.arm.move(self.up() - self.down())
