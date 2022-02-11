import commands2
import wpimath.controller

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class TurnToAngle(commands2.PIDCommand):

    def __init__(self, drive: DriveSubsystem, target_angle_degrees: float) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                DriveConstants.TURN_DEGREES_P,
                DriveConstants.TURN_DEGREES_I,
                DriveConstants.TURN_DEGREES_D
            ),
            drive.get_heading,
            target_angle_degrees,
            # Turn the robot with PID loop output
            lambda output: drive.arcade_drive(0, output),
            [drive]
        )

        self.controller = self.getController()
        self.controller.enableContinuousInput(-180, 180)
        self.controller.setTolerance(DriveConstants.TURN_TOLERANCE_DEGREES)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
