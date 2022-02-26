import commands2

from subsystems.arm_subsystem import ArmSubsystem


class SetArmHeight(commands2.InstantCommand):

    def __init__(self, arm: ArmSubsystem, height: float):
        super().__init__(lambda: arm.set_height(height), [arm])
