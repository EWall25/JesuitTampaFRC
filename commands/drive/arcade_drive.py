import typing

import commands2
import wpilib
import wpimath.controller
import wpimath.filter

from subsystems.drive_subsystem import DriveSubsystem


class ArcadeDrive(commands2.CommandBase):

    def __init__(
            self, drive: DriveSubsystem,
            forward: typing.Callable[[], float],
            rotation: typing.Callable[[], float]
    ) -> None:
        super().__init__()

        self.follow_angle: float = 0
        self.last_rotation: float = 0
        self.should_reset_zero: bool = False

        self.drive = drive
        self.forward_in = forward
        self.rotation_in = rotation

        # PID controller for keeping straight
        self.controller = wpimath.controller.PIDController(0, 0, 0)

        self.filter = wpimath.filter.SlewRateLimiter(2)

        # Timer used for resetting our follow angle
        self.timer = wpilib.Timer()

        # Reserve the drive subsystem so other commands can't use it
        self.addRequirements([drive])

    def initialize(self) -> None:
        # Tell the robot where "straight" is
        self.follow_angle = self.drive.get_heading()

        # Reset the PID Controller so calculations using I and D gains are accurate
        self.controller.reset()

    def execute(self) -> None:
        forward_in = self.forward_in()
        rotation_in = self.rotation_in()
        heading = self.drive.get_heading()

        # Correct heading so the robot drives straight if the driver isn't turning the robot
        if rotation_in == 0:
            # Calculate the rotation needed to drive straight
            rotation = self.controller.calculate(heading, self.follow_angle)
        else:
            # Set the robot's rotation to the driver's input
            rotation = rotation_in

        # Limit the robot's acceleration for smoother driving
        forward = self.filter.calculate(forward_in)

        # Drive the robot
        self.drive.arcade_drive(forward, rotation)

        # We should reset the robot's perception of "straight" after turning
        if rotation_in == 0 and self.last_rotation != 0:
            self.should_reset_zero = True
            self.timer.reset()

        # Reset the robot's straight angle only after a small period of time.
        # This is to prevent inertia from turning affecting our drive.
        if self.should_reset_zero and self.timer.get() > 0.5:
            self.follow_angle = heading
            self.should_reset_zero = False

        self.last_rotation = rotation_in

    def end(self, interrupted: bool) -> None:
        # Stop the drive motors
        self.drive.stop()
