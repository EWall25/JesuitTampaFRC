import commands2

from subsystems.arm_subsystem import ArmSubsystem


class SetArmSpeed(commands2.InstantCommand):

    def __init__(self, arm: ArmSubsystem, height: float):
        """
        Sets the speed of the arm.
        :param arm: The arm subsystem
        :param height: The speed to run the arm at as a PWM value. Ranges 0-1.
        """

        super().__init__(lambda: arm.set_speed(height), [arm])
