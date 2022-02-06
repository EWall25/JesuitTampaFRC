import typing

import commands2
import wpimath.filter

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class DefaultDrive(commands2.CommandBase):

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
        self.filter = wpimath.filter.SlewRateLimiter(DriveConstants.MAX_ACCELERATION_PER_SECOND)

        self.addRequirements([drive])

    def execute(self) -> None:
        self.drive.arcade_drive(self.filter.calculate(self.forward()), self.rotation())
