import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class LockArm(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem):
        super().__init__()

        self.arm = arm
        self.power = 0

        self.addRequirements([arm])

    def initialize(self) -> None:
        self.power = self.arm.get_power()

        # Update the dashboard
        wpilib.SmartDashboard.putBoolean("Arm Locked?", True)

    def execute(self) -> None:
        self.arm.set_power(self.power)

    def end(self, interrupted: bool) -> None:
        # Update the dashboard
        wpilib.SmartDashboard.putBoolean("Arm Locked?", False)
