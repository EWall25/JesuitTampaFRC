import typing

import commands2
import wpilib


class ReplayCommand(commands2.CommandBase):
    index = 0

    def __init__(self, file_path: str, callback: typing.Callable[[str], None], requirements: typing.List[commands2.Subsystem]):
        super().__init__()

        self.callback = callback

        try:
            with open(file_path, "r") as file:
                content = file.read()
                self.instructions = content.splitlines(False)
        except FileNotFoundError:
            if not wpilib.DriverStation.isFMSAttached():
                print("Action replay file not found!")

            self.instructions = []

        self.addRequirements(requirements)

    def execute(self) -> None:
        try:
            value = self.instructions[self.index]
        except IndexError:
            if not wpilib.DriverStation.isFMSAttached():
                print("ReplayCommand has overrun the instruction set!")
            return

        self.callback(value)

        self.index += 1

    def end(self, interrupted: bool) -> None:
        if not wpilib.DriverStation.isFMSAttached():
            print(f"ReplayCommand ended after looping through {len(self.instructions)} values.")

    def isFinished(self) -> bool:
        return self.index > len(self.instructions) - 1


