import wpilib
from wpilib.interfaces import GenericHID

import util
from constants import *

from commands.default_drive import DefaultDrive

from subsystems.drive_subsystem import DriveSubsystem


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.stick = wpilib.XboxController(DRIVER_CONTROLLER_PORT)

        # Subsystems
        self.drive = DriveSubsystem()

        # Setup default drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                lambda: util.deadband(-self.stick.getY(GenericHID.Hand.kLeftHand), 0.03),
                lambda: util.deadband(self.stick.getX(GenericHID.Hand.kRightHand), 0.03),
                self.drive
            )
        )