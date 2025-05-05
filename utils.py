def map_trigger_value(value):
    return int((value + 1) * 15) if -1 <= value <= 1 else 0

def vel_limit(value):
    if value >= 30.0:
        value = 30.0
    elif value <= 0.0:
        value = 0.0
    return value

def angle_limit(value):
    if value >= 30.0:
        value = 30.0
    elif value <= 0.0:
        value = 0.0
    return value