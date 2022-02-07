import commands2
import wpilib

from subsystems.drive_subsystem import DriveSubsystem


class TurnToAngleSimpleP(commands2.CommandBase):
    def __init__(self, drive: DriveSubsystem, target_angle: float) -> None:
        super().__init__()

        self.drive = drive
        self.goal = target_angle
        self.kP = 0.1
        self.tolerance = 3

        self.high = target_angle + self.tolerance
        self.low = target_angle - self.tolerance
    
    def execute(self) -> None:
        error = self.goal - self.drive.get_heading()
        output = self.kP * error

        self.drive.arcade_drive(0, output)
    
    def isFinished(self) -> bool:
        heading = self.drive.get_heading()
        return self.low < heading < self.high

