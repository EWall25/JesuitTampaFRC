import typing

import wpilib
import commands2

from robot_container import RobotContainer


class StealthTigersRobot(commands2.TimedCommandRobot):

    autonomous_command: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        self.container = RobotContainer()


if __name__ == "__main__":
    wpilib.run(StealthTigersRobot)
