import commands2
import wpilib
import wpilib.controller

from constants import *

from subsystems.drive_subsystem import DriveSubsystem


class TurnToAngle(commands2.PIDCommand):
    def __init__(self, drive: DriveSubsystem, target_angle_degrees: float) -> None:
        super().__init__(
            wpilib.controller.PIDController(TURN_P, TURN_I, TURN_D),
            drive.get_heading,
            target_angle_degrees,
            lambda output: drive.arcade_drive(0, output),
            [drive]
        )

        self.controller = self.getController()
        self.controller.enableContinuousInput(-180, 180)
        self.controller.setTolerance(TURN_TOLERANCE_DEGREES)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
