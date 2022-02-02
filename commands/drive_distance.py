import commands2
import wpimath.controller

from subsystems.drive_subsystem import DriveSubsystem


class DriveDistance(commands2.PIDCommand):

    def __init__(self, drive: DriveSubsystem, inches: float):
        super().__init__(
            wpimath.controller.PIDController(1, 0, 0),
            drive.get_average_distance,
            inches,
            lambda output: drive.arcade_drive(output, 0),
            [drive]
        )

        self.controller = self.getController()
        self.controller.setTolerance(3)

    def isFinished(self) -> bool:
        return self.controller.atSetpoint()
