import typing

import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class PowerControlArm(commands2.CommandBase):

    def __init__(
            self, arm: ArmSubsystem,
            movement: typing.Callable[[], float]
    ) -> None:
        """
        Drive the arm with joysticks/triggers, directly setting the arm's speed.
        :param arm: The arm subsystem
        :param movement: A function returning the intended movement of the arm. Ranges 0-1.
        This might not be the power being fed to the motor.
        """

        super().__init__()

        self.arm = arm
        self.movement = movement

        self.last_movement = 0
        self.movement_just_went_zero = False
        self.timer = wpilib.Timer()

        self.addRequirements([arm])

    def execute(self) -> None:
        movement = self.movement()

        # TODO: Make all these numbers constants
        # Reduce the power when moving up, but not when driving the motor downwards
        movement *= 0.75 if movement > 0 else 1

        # Start the timer when controls drop out
        if movement == 0 and self.last_movement > 0:
            self.timer.reset()
            self.movement_just_went_zero = True

        # Check if we are currently controlling the robot, or just were
        if movement != 0 or (self.timer.get() < 1.75 and self.movement_just_went_zero):
            # Add holding power to the power being fed to the motor
            movement += 0.25

        # If the timer has been running for more than X amount of seconds,
        # stop power the motor to prevent burnout
        if self.timer.get() > 1.75:
            self.movement_just_went_zero = False

        self.last_movement = movement

        self.arm.set_power(movement)
