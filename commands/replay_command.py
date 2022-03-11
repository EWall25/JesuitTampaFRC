import typing

import commands2
import wpilib


class ReplayCommand(commands2.CommandBase):
    index = 0

    def __init__(self, file_path: str, callback: typing.Callable[[float], None], requirements: typing.List[commands2.Subsystem]):
        super().__init__()

        self.callback = callback
        self.instructions: typing.List[float] = []

        try:
            with open(file_path, "r") as file:
                content = file.read().splitlines()
                for line in content:
                    try:
                        self.instructions.append(float(line))
                    except ValueError:
                        pass
        except FileNotFoundError:
            if not wpilib.DriverStation.isFMSAttached():
                print("Action replay file not found!")

        self.addRequirements(requirements)

    def initialize(self) -> None:
        self.index = 0

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


