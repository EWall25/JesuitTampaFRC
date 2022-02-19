import commands2
import commands2.button
import wpilib

from commands.arm.default_arm import DefaultArm
from commands.drive.default_drive import DefaultDrive
from commands.drive.drive_distance import DriveDistance
from commands.drive.drive_distance_simple import DriveDistanceSimple
from commands.drive.drive_distance_straight import DriveDistanceStraight
from commands.drive.timed_drive import TimedDrive
from commands.drive.turn_to_angle import TurnToAngle
from constants import DriveConstants, DriverStationConstants
from subsystems.arm_subsystem import ArmSubsystem
from subsystems.drive_subsystem import DriveSubsystem
from util import Units


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.stick = wpilib.PS4Controller(DriverStationConstants.DRIVER_CONTROLLER_PORT)

        # Timer
        self.timer = wpilib.Timer()

        # Subsystems
        self.drive = DriveSubsystem()
        self.arm = ArmSubsystem()

        # Disable arm limits TODO
        self.arm.set_safety(False)

        # Add subsystems to the dashboard
        # wpilib.SmartDashboard.putData(self.drive)
        # wpilib.SmartDashboard.putData(self.arm)

        # Autonomous routines

        # Competition autonomous routine
        self.competition_auto = commands2.SequentialCommandGroup(
            # TODO: Raise Arm
            # TODO: Lower Arm
            # Drive out of the tarmack
            DriveDistanceSimple(self.drive, Units.feet_to_metres(-7.5))
        )

        self.drive_straight_auto = DriveDistanceStraight(self.drive, 5, max_speed=0.3, reset_heading=True)

        # Auto routine chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the auto command chooser
        self.chooser.setDefaultOption("Competition", self.competition_auto)
        self.chooser.addOption("Drive Straight", self.drive_straight_auto)
        self.chooser.addOption("Nothing", commands2.InstantCommand())

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configure_button_bindings()

        # Setup default drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                # Set the forward speed to the left stick's Y axis. If the right bumper is pressed, speed up
                lambda: -self.stick.getRawAxis(DriverStationConstants.DRIVE_STICK) * (
                    DriveConstants.TELEOP_BOOST_DRIVE_SPEED if self.stick.getR1Button()
                    else DriveConstants.TELEOP_DEFAULT_DRIVE_SPEED),
                # Set the rotation speed to the right stick's X axis.
                lambda: self.stick.getRawAxis(DriverStationConstants.TURN_STICK) * DriveConstants.TELEOP_TURN_SPEED
            )
        )

        # Setup default arm mode
        self.arm.setDefaultCommand(
            DefaultArm(
                self.arm,
                lambda: self.stick.getRawAxis(3),
                lambda: self.stick.getRawAxis(2)
            )
        )

    def configure_button_bindings(self) -> None:
        # Turn to 0 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, self.stick.Button.kTriangle).whenPressed(
            TurnToAngle(self.drive, 0.0).withTimeout(5)  # Timeout the command if it hasn't completed after 5 seconds
        )
        # Turn to 90 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, self.stick.Button.kCircle).whenPressed(
            TurnToAngle(self.drive, 90.0).withTimeout(5)
        )
        # Turn to 180 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, self.stick.Button.kCross).whenPressed(
            TurnToAngle(self.drive, 179.9).withTimeout(5)
        )
        # Turn to -90 degrees (90 degrees left) when a button is pushed
        commands2.button.JoystickButton(self.stick, self.stick.Button.kSquare).whenPressed(
            TurnToAngle(self.drive, -90.0).withTimeout(5)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
