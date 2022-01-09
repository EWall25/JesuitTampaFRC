import commands2
import wpilib
from wpilib.interfaces import GenericHID

import util
from constants import *

from commands.default_drive import DefaultDrive
from commands.timed_drive import TimedDrive

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
        self.auto = TimedDrive(5, 0.7, self.drive, self.timer)

        # Setup default drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                lambda: util.deadband(-self.stick.getY(GenericHID.Hand.kLeftHand), 0.03),
                lambda: util.deadband(self.stick.getX(GenericHID.Hand.kRightHand), 0.03),
                self.drive
            )
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self.auto
