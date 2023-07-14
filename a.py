def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:

    a = (val - old_min) / (old_max - old_min)
    return a * (new_max - new_min) + new_min
