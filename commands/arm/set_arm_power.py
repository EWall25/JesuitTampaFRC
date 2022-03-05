import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class SetArmPower(commands2.CommandBase):

    def __init__(self, arm: ArmSubsystem, power: float, hold_seconds: float):
        # TODO: FIX ALL THIS
        """
        Gradually set the arm's power.

        :param arm: The arm subsystem.
        :param power: The desired power of the arm
        :param hold_seconds: Amount of time in seconds the arm will be held in the target power.
        The command will last longer than this time.
        """

        super().__init__()

        self.arm = arm
        self.goal = power
        self.hold_time = hold_seconds

        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.stop()

    def execute(self) -> None:
        current_power = self.arm.get_power()
        time = self.timer.get()

        if current_power != self.goal:
            # Increase the power until the goal is reached
            power = min(current_power + 0.05, self.goal)
        elif time < 0.02:
            self.timer.reset()
            self.timer.start()
            power = self.goal
        elif time < 1.75:
            # Hold at the target power
            power = self.goal
        else:
            # Power low enough to drop slowly
            power = 0.25

        self.arm.set_power(power)

    def isFinished(self) -> bool:
        time = self.timer.get()
        return time > self.hold_time + 1.75

