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
            # Output will be in volts, so drive both motors with the output value
            lambda output: drive.tank_drive_volts(output, output),
            [drive]
        )

        self.controller = self.getController()
        self.controller.setTolerance(DriveConstants.LINEAR_TOLERANCE_METRES)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
