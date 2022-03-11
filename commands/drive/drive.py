import commands2

from subsystems.drive_subsystem import DriveSubsystem


class Drive(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, forward: float, rotation: float):
        super().__init__()

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

    def execute(self) -> None:
        self.drive.arcade_drive(self.forward, self.rotation)

    def end(self, interrupted: bool) -> None:
        self.drive.stop()