import os.path
import typing

import wpilib


class Logger:
    def __init__(self, prefix: str):
        if wpilib.RobotBase.isReal():
            self.LOG_DIRECTORY = "/u/logs"
        else:
            self.LOG_DIRECTORY = os.environ.get("LOG_DIR_PATH")

        self.prefix = prefix
        self.data: typing.List[str] = []

    def log(self, value: str):
        self.data.append(value)

    def clear(self):
        """
        Clear any currently logged data out.
        """

        self.data.clear()

    def flush(self):
        """
        Write the logged data to a file. The file's name will be the prefix appended to the current time.
        """

        filename = f"{self.prefix}{str(wpilib.getTime())}.txt"
        path = os.path.join(self.LOG_DIRECTORY, filename)

        try:
            with open(path, "w") as file:
                file.writelines(s + "\n" for s in self.data)
        except FileNotFoundError:
            if not wpilib.DriverStation.isFMSAttached():
                print("File can't be created! Logging unsuccessful!")
