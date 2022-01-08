import commands2
import typing

from subsystems.drive_subsystem import DriveSubsystem


class DefaultDrive(commands2.CommandBase):

    def __init__(
            self, forward: typing.Callable[[], float],
            rotation: typing.Callable[[], float],
            drive: DriveSubsystem
    ) -> None:
        super().__init__()

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        self.addRequirements([drive])

    def execute(self) -> None:
        self.drive.arcade_drive(self.forward(), self.rotation())
