import typing

import commands2
import wpimath.filter

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class ArcadeDrive(commands2.CommandBase):

    def __init__(
            self, drive: DriveSubsystem,
            forward: typing.Callable[[], float],
            rotation: typing.Callable[[], float],
    ) -> None:
        """
        Arcade drive mode.
        :param forward: A function returning the intended forward speed. Ranges -1 to 1.
        :param rotation: A function returning the intended rotation speed. Ranges -1 to 1.
        :param drive: The drive subsystem.
        """

        super().__init__()

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        # A SlewRateLimiter limits our acceleration in order to make the robot move smoother
        self.filter = wpimath.filter.SlewRateLimiter(DriveConstants.TELEOP_DRIVE_ACCELERATION_PER_SECOND)

        self.last_forward = 0

        self.addRequirements([drive])

    def execute(self) -> None:
        forward = self.forward()
        rotation = self.rotation()

        # Make changing directions quicker
        if (forward < 0) != (self.last_forward < 0):
            self.filter.reset(forward)

        self.drive.arcade_drive(self.filter.calculate(forward), rotation)

        self.last_forward = forward
