import commands2
import commands2.button
import wpilib
from wpilib.interfaces import GenericHID

import util
from constants import *

from commands.default_drive import DefaultDrive
from commands.timed_drive import TimedDrive
from commands.turn_to_angle import TurnToAngle

from subsystems.drive_subsystem import DriveSubsystem


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.stick = wpilib.XboxController(DRIVER_CONTROLLER_PORT)

        # Timer
        self.timer = wpilib.Timer()

        # Subsystems
        self.drive = DriveSubsystem()

        # Autonomous routines

        # Auto routine which drives forwards for 5 seconds, then stops
        self.timedAuto = TimedDrive(self.drive, self.timer, 5, 0.7)

        # Auto routine chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the auto command chooser
        self.chooser.setDefaultOption("Timed Auto", self.timedAuto)
        self.chooser.addOption("Nothing", commands2.InstantCommand())

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configure_button_bindings()

        # Setup default drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                # Apply a deadzone to the input to prevent controller drift
                # If the right bumper is pressed, increase the movement speed
                lambda: util.deadband(-self.stick.getY(GenericHID.Hand.kLeftHand), 0.03)
                        * (BOOST_DRIVE_SPEED if self.stick.getBumper(GenericHID.Hand.kRightHand)
                        else DEFAULT_DRIVE_SPEED),
                lambda: util.deadband(self.stick.getX(GenericHID.Hand.kRightHand), 0.03)
                        * (BOOST_TURN_SPEED if self.stick.getBumper(GenericHID.Hand.kRightHand)
                        else DEFAULT_TURN_SPEED),
            )
        )

    def configure_button_bindings(self) -> None:
        # Turn to 0 degrees when a button is pushed
        commands2.button.JoystickButton(self.stick, TURN_TO_ZERO_BUTTON).whenPressed(
            TurnToAngle(0.0, self.drive).withTimeout(5)  # Timeout the command it it hasn't completed after 5 seconds
        )
        commands2.button.JoystickButton(self.stick, TURN_TO_NINETY_BUTTON).whenPressed(
            TurnToAngle(90.0, self.drive).withTimeout(5)
        )
        commands2.button.JoystickButton(self.stick, TURN_TO_ONE_EIGHTY_BUTTON).whenPressed(
            TurnToAngle(179.9, self.drive).withTimeout(5)
        )
        commands2.button.JoystickButton(self.stick, TURN_TO_NEGATIVE_NINETY_BUTTON).whenPressed(
            TurnToAngle(-90.0, self.drive).withTimeout(5)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
