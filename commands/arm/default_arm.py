import typing

import commands2

from subsystems.arm_subsystem import ArmSubsystem


class DefaultArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            rotation: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers.
        :param arm: The arm subsystem
        :param rotation: A function returning the power to the arm motor. Ranges from -1 to 1.
        """

        super().__init__()

        self.arm = arm
        self.rotation = rotation

        self.addRequirements([arm])

    def execute(self) -> None:
        self.arm.move(self.rotation())
