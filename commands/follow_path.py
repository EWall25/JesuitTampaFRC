import commands2
import wpimath.trajectory
import wpimath.kinematics
import wpimath.controller
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.drive_subsystem import DriveSubsystem


class FollowPath(commands2.RamseteCommand):

    def __init__(self, drive: DriveSubsystem):
        self.drive = drive

        # Configure the trajectory
        config = wpimath.trajectory.TrajectoryConfig(5, 2)
        config.setKinematics(drive.kinematics)

        # Create an example trajectory to follow
        self.trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing 0 degrees
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through two waypoints, making an 's' curve path
            [
                Translation2d(1, 1),
                Translation2d(2, -1)
            ],
            # End 3 metres ahead of where we started, facing 0 degrees
            Pose2d(3, 0, Rotation2d(0)),
            config
        )

        super().__init__(
            trajectory=self.trajectory,
            pose=drive.get_pose,
            controller=wpimath.controller.RamseteController(),
            feedforward=wpimath.controller.SimpleMotorFeedforwardMeters(1, 1, 1),
            kinematics=drive.kinematics,
            wheelSpeeds=drive.get_wheel_speeds,
            leftController=wpimath.controller.PIDController(1, 0, 0),
            rightController=wpimath.controller.PIDController(1, 0, 0),
            output=drive.tank_drive_volts,
            requirements=[drive]
        )

    def initialize(self) -> None:
        # Set the robot's pose to the trajectory's starting pose
        self.drive.reset_pose(self.trajectory.initialPose())
