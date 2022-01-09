import commands2
import wpilib

from subsystems.drive_subsystem import DriveSubsystem


class TimedDrive(commands2.CommandBase):

    def __init__(self, seconds: float, speed: float, drive: DriveSubsystem, timer: wpilib.Timer) -> None:
        super().__init__()

        self.drive = drive
        self.time = seconds
        self.speed = speed
        self.timer = timer

        self.addRequirements([drive])

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.drive.arcade_drive(self.speed, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.stop()

    def isFinished(self) -> bool:
        return self.timer.get() > self.time
