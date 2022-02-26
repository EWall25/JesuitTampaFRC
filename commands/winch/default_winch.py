import typing

import commands2

from subsystems.winch_subsystem import WinchSubsystem


class DefaultWinch(commands2.CommandBase):

    def __init__(
            self, winch: WinchSubsystem,
            power: typing.Callable[[], float],
    ) -> None:
        super().__init__()

        self.winch = winch
        self.power = power

        self.addRequirements([winch])

    def execute(self) -> None:
        power = max(self.power(), 0)
        self.winch.drive(power)
