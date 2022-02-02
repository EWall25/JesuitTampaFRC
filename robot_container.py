import commands2
import commands2.button
import wpilib

from commands.default_drive import DefaultDrive
from commands.drive_distance import DriveDistance
from commands.timed_drive import TimedDrive
from commands.turn_to_angle import TurnToAngle
from constants import *
from subsystems.drive_subsystem import DriveSubsystem


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.stick = wpilib.PS4Controller(DRIVER_CONTROLLER_PORT)

        # Timer
        self.timer = wpilib.Timer()

        # Subsystems
        self.drive = DriveSubsystem()

        # Autonomous routines

        # Auto routine which drives forwards for 5 seconds, then stops
        self.timedAuto = TimedDrive(self.drive, self.timer, 5, 0.7)

        # Auto routine which drives forwards 5 feet, then stops
        self.distanceAuto = DriveDistance(self.drive, 5 * 12)

        # Auto routine which turns to 90 degrees
        self.angleAuto = TurnToAngle(self.drive, 90)

        # Auto routine chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the auto command chooser
        self.chooser.setDefaultOption("Timed Auto", self.timedAuto)
        self.chooser.addOption("Distance Auto", self.distanceAuto)
        self.chooser.addOption("Angle Auto", self.angleAuto)
        self.chooser.addOption("Nothing", commands2.InstantCommand())

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configure_button_bindings()

        # Setup default drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                # Set the forward speed to the left stick's Y axis. If the right bumper is pressed, speed up
                lambda: -self.stick.getLeftY() * (BOOST_DRIVE_SPEED if self.stick.getR1Button() else DEFAULT_DRIVE_SPEED),
                # Set the rotation speed to the right stick's X axis. If the right bumper is pressed, speed up
                lambda: self.stick.getRightX() * (BOOST_TURN_SPEED if self.stick.getR1Button() else DEFAULT_TURN_SPEED)
            )
        )

    def configure_button_bindings(self) -> None:
        # Turn to 0 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, TURN_TO_ZERO_BUTTON).whenPressed(
            TurnToAngle(self.drive, 0.0).withTimeout(5)  # Timeout the command if it hasn't completed after 5 seconds
        )
        # Turn to 90 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, TURN_TO_NINETY_BUTTON).whenPressed(
            TurnToAngle(self.drive, 90.0).withTimeout(5)
        )
        # Turn to 180 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, TURN_TO_ONE_EIGHTY_BUTTON).whenPressed(
            TurnToAngle(self.drive, 179.0).withTimeout(5)
        )
        # Turn to -90 degrees (90 degrees left) when a button is pushed
        commands2.button.JoystickButton(self.stick, TURN_TO_NEGATIVE_NINETY_BUTTON).whenPressed(
            TurnToAngle(self.drive, -90.0).withTimeout(5)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
