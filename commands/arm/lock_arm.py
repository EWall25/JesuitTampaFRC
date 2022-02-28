import typing

import commands2

from subsystems.arm_subsystem import ArmSubsystem


class LockArm(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem):
        super().__init__()

        self.arm = arm
        self.power = 0

        self.addRequirements([arm])

    def initialize(self) -> None:
        self.power = self.arm.get_speed()

    def execute(self) -> None:
        self.arm.set_speed(self.power)
