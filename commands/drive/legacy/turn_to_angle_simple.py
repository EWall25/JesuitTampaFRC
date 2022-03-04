import commands2

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class TurnToAngleSimple(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, target_angle: float) -> None:
        super().__init__()

        self.drive = drive
        self.goal = target_angle

        self.addRequirements([drive])

    def initialize(self) -> None:
        self.drive.reset_heading()

    def execute(self) -> None:
        heading = -self.drive.get_heading()
        turn_right = self.goal > heading
        if turn_right:
            self.drive.arcade_drive(0, 0.3)
        else:
            self.drive.arcade_drive(0, -0.3)

    def end(self, interrupted: bool) -> None:
        self.drive.stop()

    def isFinished(self) -> bool:
        lower = self.goal - DriveConstants.TURN_TOLERANCE_DEGREES
        upper = self.goal + DriveConstants.TURN_TOLERANCE_DEGREES

        heading = -self.drive.get_heading()

        return lower < heading < upper
