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
        # When moving up, some value is already being added by default
        if movement > 0:
            power = movement * 0.75
        else:
            power = movement

        # Start the timer when controls drop out
        if movement == 0 and self.last_movement > 0:
            self.timer.reset()
            self.timer.start()
            self.movement_just_went_zero = True

        time = self.timer.get()

        # Check if we are currently controlling the robot, or just were
        if movement > 0 or (time < 1.75 and self.movement_just_went_zero and movement == 0):
            # Add holding power to the power being fed to the motor
            power += 0.25

        # If the timer has been running for more than X amount of seconds,
        # stop power the motor to prevent burnout
        if time > 1.75:
            self.movement_just_went_zero = False

        self.last_movement = movement

        self.arm.set_power(power)
