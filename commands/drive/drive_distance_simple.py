import commands2
from constants import DriveConstants

from subsystems.drive_subsystem import DriveSubsystem


class DriveDistanceSimple(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, metres: float, speed: float = 0.5) -> None:
        """
        Drive the robot a distance.
        :param drive: The drivetrain subsystem
        :param metres: The distance to drive the robot
        :param speed: The drive speed. Ranges 0-1
        """
        super().__init__()

        self.drive = drive
        self.goal = metres
        self.speed = speed

        self.addRequirements([drive])

    def initialize(self) -> None:
        self.drive.reset_encoders()

    def execute(self) -> None:
        distance = self.drive.get_average_distance()
        drive_forward = self.goal > distance
        print(f"{distance} :: {drive_forward}")
        if drive_forward:
            self.drive.arcade_drive(self.speed, 0)
        else:
            self.drive.arcade_drive(-self.speed, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.stop()

    def isFinished(self) -> bool:
        lower = self.goal - DriveConstants.DRIVE_TOLERANCE_METRES
        upper = self.goal + DriveConstants.DRIVE_TOLERANCE_METRES

        distance = self.drive.get_average_distance()

        return lower < distance < upper
