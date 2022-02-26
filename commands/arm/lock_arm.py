import typing

import commands2

from subsystems.arm_subsystem import ArmSubsystem


class LockArm(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem, holding_value: typing.Callable[[], float]):
        super().__init__()

        self.arm = arm
        self.holding_value = holding_value
        self.value = 0

        self.addRequirements([arm])

    def initialize(self) -> None:
        self.value = self.holding_value()

    def execute(self) -> None:
        self.arm.set_height(self.value)
