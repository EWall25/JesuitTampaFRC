def deadband(value: float, lower_limit: float) -> float:
    return value if abs(value) > lower_limit else 0
