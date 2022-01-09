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
        self.timedAuto = TimedDrive(5, 0.7, self.drive, self.timer)

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
                lambda: util.deadband(-self.stick.getY(GenericHID.Hand.kLeftHand), 0.03),
                lambda: util.deadband(self.stick.getX(GenericHID.Hand.kRightHand), 0.03),
                self.drive
            )
        )

    def configure_button_bindings(self) -> None:
        commands2.button.JoystickButton(self.stick, TURN_TO_ZERO_BUTTON).whenPressed(
            TurnToAngle(0.0, self.drive).withTimeout(5)
        )
        commands2.button.JoystickButton(self.stick, TURN_TO_NINETY_BUTTON).whenPressed(
            TurnToAngle(90.0, self.drive).withTimeout(5)
        )
        commands2.button.JoystickButton(self.stick, TURN_TO_NEGATIVE_NINETY_BUTTON).whenPressed(
            TurnToAngle(-90.0, self.drive).withTimeout(5)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()
