import commands2
import wpimath.controller

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class TurnToAngle(commands2.PIDCommand):
    def __init__(self, drive: DriveSubsystem, target_angle_degrees: float) -> None:
        super().__init__(
            wpimath.controller.PIDController(DriveConstants.P_GAIN, DriveConstants.I_GAIN, DriveConstants.D_GAIN),
            drive.get_heading,
            target_angle_degrees,
            # Output will be in volts, so rotate the robot by driving positive on one motor and negative on the other
            lambda output: drive.tank_drive_volts(output, -output),
            [drive]
        )

        self.controller = self.getController()
        self.controller.enableContinuousInput(-180, 180)
        self.controller.setTolerance(DriveConstants.ANGULAR_TOLERANCE_DEGREES)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
