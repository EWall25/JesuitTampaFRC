import commands2

from constants import WinchConstants
from subsystems.winch_subsystem import WinchSubsystem


class EngageWinch(commands2.WaitCommand):

    def __init__(self, winch: WinchSubsystem) -> None:
        """
        Pull the winch tight.
        :param winch: The winch subsystem
        """

        # Drive the winch for a certain amount of time
        super().__init__(WinchConstants.WINCH_DRIVE_TIME_SECONDS)

        self.winch = winch

        self.addRequirements([winch])

    def execute(self) -> None:
        # Drive the winch
        self.winch.drive()

    def end(self, interrupted: bool) -> None:
        # Stop the winch motor
        self.winch.stop()
