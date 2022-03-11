import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class LowerArm(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem):
        super().__init__()

        self.arm = arm
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.arm.set_power(0.2)

    def end(self, interrupted: bool) -> None:
        self.arm.set_power(0)

    def isFinished(self) -> bool:
        return self.timer.get() > 1.75
