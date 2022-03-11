def clamp(number: float, min_value: float, max_value: float) -> float:
    return max(min(number, max_value), min_value)


def deadband(number: float, deadband: float) -> float:
    if abs(number) < deadband:
        return 0
    return number


class Units:

    @staticmethod
    def feet_to_metres(feet: float):
        return feet * 0.3048

    @staticmethod
    def inches_to_metres(inches: float):
        return inches * 0.0254
