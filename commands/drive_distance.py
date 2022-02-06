import commands2
import wpimath.controller

from constants import DriveConstants
from subsystems.drive_subsystem import DriveSubsystem


class DriveDistance(commands2.PIDCommand):

    def __init__(self, drive: DriveSubsystem, metres: float):
        super().__init__(
            wpimath.controller.PIDController(
                DriveConstants.P_GAIN,
                DriveConstants.I_GAIN,
                DriveConstants.D_GAIN
            ),
            drive.get_average_distance,
            metres,
            lambda output: drive.arcade_drive(output, 0),
            [drive]
        )

        self.controller = self.getController()
        self.controller.setTolerance(DriveConstants.LINEAR_TOLERANCE_METRES)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
