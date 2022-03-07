import commands2
import wpilib

from subsystems.arm_subsystem import ArmSubsystem


class SetArmPower(commands2.CommandBase):
    RAMP_TIME = 1

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
        self.timer.start()

    '''
    def execute(self) -> None:
        current_power = self.arm.get_power()
        ramp_up_time = self.ramp_up_timer.get()
        time_2 = self.timer_2.get()

        if ramp_up_time < 1:
            power = 0.5
        elif current_power != self.goal:
            # Increase/decrease the power until the goal is reached
            if current_power < self.goal:
                power = min(current_power + 0.05, self.goal)
            else:
                power = max(current_power - 0.05, self.goal)

        if self.last_power != current_power:
            self.timer_2.reset()
            self.timer_2.start()
        else:
            self.hold_flag = True

        if time_2 < self.hold_time and self.hold_flag:
            # Hold at the target power
            power = self.goal
        elif time_2 > 0.05:
            # Power low enough to drop slowly
            power = 0.25

        self.last_power = current_power

        self.arm.set_power(power)
    '''

    def execute(self):
        current_power = self.arm.get_power()
        time = self.timer.get()

        power = 0

        if time < self.RAMP_TIME:
            # Overcome friction on the motor
            power = 0.7

        if self.RAMP_TIME < time < self.hold_time + self.RAMP_TIME:
            power = self.goal

        if time > self.RAMP_TIME + self.hold_time:
            power = 0.25

        print(f"{power} :: {time}")
        self.arm.set_power(power)

    def isFinished(self) -> bool:
        time = self.timer.get()
        return time > self.hold_time + self.RAMP_TIME + 1.75     # Time holding + the ramp up time + the cooldown time

