import commands2
import commands2.button
import wpilib
from wpilib import PS4Controller

from commands.arm.arm_position_commands import RaiseArm, LowerArm
from commands.arm.default_arm import DefaultArm
from commands.drive.arcade_drive import ArcadeDrive
from commands.drive.drive_distance_simple import DriveDistanceSimple
from commands.winch.engage_winch import EngageWinch
from constants import DriveConstants, DriverStationConstants, AutoConstants
from subsystems.arm_subsystem import ArmSubsystem
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.winch_subsystem import WinchSubsystem
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
        self.winch = WinchSubsystem()

        # Add subsystems to the dashboard
        # wpilib.SmartDashboard.putData(self.drive)
        # wpilib.SmartDashboard.putData(self.arm)

        # Autonomous routines

        # Competition autonomous routine
        self.competition_auto = commands2.SequentialCommandGroup(
            # Raise the arm to the hub
            RaiseArm(self.arm).withTimeout(AutoConstants.RAISE_ARM_SECONDS),
            # Lower the arm, so we can drive away from the hub
            LowerArm(self.arm).withTimeout(AutoConstants.LOWER_ARM_SECONDS),
            # Drive out of the tarmack
            DriveDistanceSimple(self.drive, Units.feet_to_metres(-7.5), AutoConstants.DRIVE_AWAY_FROM_HUB_SPEED)
        )

        # Auto routine chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the auto command chooser
        self.chooser.setDefaultOption("Competition", self.competition_auto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configure_button_bindings()

        # Setup default drive mode
        self.drive.setDefaultCommand(
            ArcadeDrive(
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
        # Engage the winch when the top button is pressed
        commands2.button.JoystickButton(self.stick, PS4Controller.Button.kTriangle).whenPressed(
            EngageWinch(self.winch)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
