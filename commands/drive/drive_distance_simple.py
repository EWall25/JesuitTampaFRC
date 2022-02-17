import commands2
from constants import DriveConstants

from subsystems.drive_subsystem import DriveSubsystem


class DriveDistanceSimple(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, metres: float) -> None:
        super().__init__()

        self.drive = drive
        self.goal = metres

        self.addRequirements([drive])
    
    def initialize(self) -> None:
        self.drive.reset_encoders()

    def execute(self) -> None:
        distance = self.drive.get_left_wheel_distance() # TODO: Change back to get_average_distance
        drive_forward = self.goal > distance
        if drive_forward:
            self.drive.arcade_drive(0.5, 0)
        else:
            self.drive.arcade_drive(-0.5, 0)
    
    def end(self, interrupted: bool) -> None:
        self.drive.stop()
    
    def isFinished(self) -> bool:
        lower = self.goal - DriveConstants.DRIVE_TOLERANCE_METRES
        upper = self.goal + DriveConstants.DRIVE_TOLERANCE_METRES

        distance = self.drive.get_average_distance()

        return lower < distance < upper