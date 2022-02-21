import typing


class Units:

    @staticmethod
    def feet_to_metres(feet: float):
        return feet * 0.3048

    @staticmethod
    def inches_to_metres(inches: float):
        return inches * 0.0254


class Scheduler:
    tasks: list[tuple[typing.Callable, float]]

    def schedule_task(self, task: typing.Callable, time: float):
        self.tasks.append((task, time))


