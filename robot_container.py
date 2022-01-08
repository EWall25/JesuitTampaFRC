import wpilib
from wpilib.interfaces import GenericHID

from constants import *

from commands.default_drive import DefaultDrive

from subsystems.drive_subsystem import DriveSubsystem


class RobotContainer:

    def __init__(self) -> None:
        # Driver controller
        self.stick = wpilib.Joystick(DRIVER_CONTROLLER_PORT)

        # Subsystems
        self.drive = DriveSubsystem()

        # Setup defualt drive mode
        self.drive.setDefaultCommand(
            DefaultDrive(
                lambda: -self.stick.getY(GenericHID.Hand.kLeftHand),
                lambda: self.stick.getX(GenericHID.Hand.kRightHand),
                self.drive
            )
        )