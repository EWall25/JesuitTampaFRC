import commands2
import commands2.button
import wpilib

import util
from commands.arm.default_arm import DefaultArm
from commands.arm.direct_arm import DirectArm
from commands.arm.lock_arm import LockArm
from commands.arm.set_arm_height import SetArmHeight
from commands.drive.arcade_drive import ArcadeDrive
from commands.drive.drive_distance_simple import DriveDistanceSimple
from commands.winch.default_winch import DefaultWinch
from commands.winch.engage_winch import EngageWinch
from constants import DriveConstants, DriverStationConstants, AutoConstants, ArmConstants
from subsystems.arm_subsystem import ArmSubsystem
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.winch_subsystem import WinchSubsystem
from util import Units


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.driver_stick = wpilib.XboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT)

        # Arm controller
        self.arm_stick = wpilib.XboxController(DriverStationConstants.ARM_CONTROLLER_PORT)

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
            SetArmHeight(self.arm, ArmConstants.LOWER_HUB_HEIGHT_PWM),
            # Drive out of the tarmack
            DriveDistanceSimple(self.drive, Units.feet_to_metres(AutoConstants.DRIVE_AWAY_FROM_HUB_DISTANCE_FEET),
                                AutoConstants.DRIVE_AWAY_FROM_HUB_SPEED)
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
                lambda: self.driver_stick.getRawAxis(DriverStationConstants.DRIVE_STICK) * (
                    DriveConstants.TELEOP_BOOST_DRIVE_SPEED
                    if self.driver_stick.getRawButton(DriverStationConstants.SPEED_TOGGLE_BUTTON)
                    else DriveConstants.TELEOP_DEFAULT_DRIVE_SPEED),
                # Set the rotation speed to the right stick's X axis.
                lambda: self.driver_stick.getRawAxis(DriverStationConstants.TURN_STICK) * DriveConstants.TELEOP_TURN_SPEED
            )
        )

        # Setup default arm mode
        self.arm.setDefaultCommand(
            DirectArm(
                self.arm,
                # Use a stick to control arm movement
                lambda: util.deadband(
                    -self.arm_stick.getRawAxis(DriverStationConstants.ARM_AXIS),
                    0.1
                )
            )
        )

        self.winch.setDefaultCommand(
            DefaultWinch(
                self.winch,
                lambda: -self.arm_stick.getRightY() * 0.3
            )
        )

    def configure_button_bindings(self) -> None:
        """
        Sets up joystick buttons.
        """

        # Engage the winch when the top button is pressed
        commands2.button.JoystickButton(self.arm_stick, DriverStationConstants.WINCH_BUTTON).whenPressed(
            EngageWinch(self.winch)
        )
        # Lock the arm in place
        commands2.button.JoystickButton(self.arm_stick, DriverStationConstants.LOCK_ARM_BUTTON).toggleWhenPressed(
            LockArm(self.arm)
        )
        '''
        commands2.button.JoystickButton(self.arm_stick, DriverStationConstants.RAMP_BUTTON).whenPressed(
            SetArmHeight(self.arm, ArmConstants.RAMP_HEIGHT_PWM)
        )
        commands2.button.JoystickButton(self.arm_stick, DriverStationConstants.LOWER_HUB_BUTTON).whenPressed(
            SetArmHeight(self.arm, ArmConstants.LOWER_HUB_HEIGHT_PWM)
        )
        commands2.button.JoystickButton(self.arm_stick, DriverStationConstants.ARM_MODE_BUTTON).toggleWhenPressed(
            DirectArm(
                self.arm,
                lambda: util.deadband(
                    -self.arm_stick.getRawAxis(DriverStationConstants.ARM_AXIS),
                    0.1
                )
            )
        )
        '''

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
