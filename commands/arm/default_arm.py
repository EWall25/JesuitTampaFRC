import typing

import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class DefaultArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers, adding to or subtracting from the arm's height.
        :param arm: The arm subsystem
        :param movement: A function returning how much should be added to the arm's height.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement

        self.addRequirements([arm])

    def initialize(self) -> None:
        # Tell the driver which mode the arm is being controlled with
        wpilib.SmartDashboard.putString("Arm Mode", "Height")

    def execute(self) -> None:
        # Add the movement value to the arm's height
        self.arm.move(self.movement())
