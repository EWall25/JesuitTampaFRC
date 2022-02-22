import commands2
import wpilib
import wpimath.controller

from subsystems.drive_subsystem import DriveSubsystem


class DriveDistanceStraight(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, metres: float, follow_angle: float = 0, max_speed: float = 1,
                 reset_heading: bool = False, finish: bool = True):
        super().__init__()

        self.drive = drive
        self.goal = metres
        self.follow = follow_angle
        self.max_speed = max_speed
        self.reset_heading = reset_heading
        self.finish = finish

        # Reserve the drive subsystem so only this command can use it
        self.addRequirements([drive])

        # PID Controller for driving a certain amount of distance
        self.distance_controller = wpimath.controller.PIDController(1, 0, 0)

        # Tell the distance controller how far we want to drive
        self.distance_controller.setSetpoint(self.goal)

        # PID Controller for keeping straight
        self.angle_controller = wpimath.controller.PIDController(0, 0, 0)
        self.angle_controller.enableContinuousInput(-180, 180)

        # Tell the drive-straight controller the angle we want to keep
        self.angle_controller.setSetpoint(self.follow)

    def initialize(self) -> None:
        # Reset our distance reading
        self.drive.reset_encoders()

        if self.reset_heading:
            self.drive.reset_heading()

        # Reset our PID controllers so they return accurate values
        self.distance_controller.reset()
        self.angle_controller.reset()

        # Put our PID controllers on the Smart Dashboard for tuning
        wpilib.SmartDashboard.putData("Distance Controller", self.distance_controller)
        wpilib.SmartDashboard.putData("Angle Controller", self.angle_controller)

    def execute(self) -> None:
        # Calculate forward/backward drive speed
        distance = self.drive.get_average_distance()
        forward = self.distance_controller.calculate(distance)

        # Stop the robot from exceeding the max speed
        forward = min(forward, self.max_speed)

        # Calculate rotational speed
        heading = self.drive.get_heading()
        rotate = self.angle_controller.calculate(heading)

        # Drive the robot while following the angle
        self.drive.arcade_drive(forward, rotate)

    def end(self, interrupted: bool) -> None:
        # Stop the motors
        self.drive.stop()

        # Remove the PID Controllers from the Smart Dashboard
        wpilib.SmartDashboard.delete("Distance Controller")
        wpilib.SmartDashboard.delete("Angle Controller")

    def isFinished(self) -> bool:
        # The command has finished once we're at the target distance
        return self.finish and self.distance_controller.atSetpoint()
