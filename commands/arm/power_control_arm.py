import typing

import commands2
import wpilib
import wpimath.filter

from constants import ArmConstants
from subsystems.arm_subsystem import ArmSubsystem


class PowerControlArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers, directly setting the arm's speed.
        :param arm: The arm subsystem
        :param movement: A function returning the robot arm's intended speed.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement
        self.filter = wpimath.filter.SlewRateLimiter(.25)

        self.addRequirements([arm])

    def execute(self) -> None:
        desired_speed = self.movement()

        # Limit the arm speed going down
        '''
        if desired_speed < self.speed:
            # Reduce the speed gradually by 0.05
            self.speed = max(desired_speed, desired_speed - 0.05)
        else:
            # If we want to move up, set the motor speed to the axis
            self.speed = desired_speed
        '''

        # Set the motor speed
        self.arm.set_speed(self.movement())
