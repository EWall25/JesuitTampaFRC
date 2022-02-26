import commands2

from subsystems.arm_subsystem import ArmSubsystem


class SetArmHeight(commands2.InstantCommand):

    def __init__(self, arm: ArmSubsystem, height: float):
        """

        :param arm: The arm subsystem
        :param height: The height to move the arm to as a PWM value. Ranges 0-1.
        """

        # TODO: Gradually decrease height
        super().__init__(lambda: arm.set_height(height), [arm])
