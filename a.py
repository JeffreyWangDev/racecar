def pid_control(
    p_gain,
    i_gain,
    d_gain,
    target_val,
    current_val,
    accumulated_error,
    last_error,
    dt
):

    error = target_val - current_val
    #change dt with time of car
    # Update the accumulated error
    accumulated_error += error * dt


    delta_error = (error - last_error) / dt

    p_term = p_gain * error
    i_term = i_gain * accumulated_error
    d_term = d_gain * delta_error

    return p_term + i_term + d_term, accumulated_error, error

def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:

    a = (val - old_min) / (old_max - old_min)
    return a * (new_max - new_min) + new_min

def test_pid_control():
    p_gain = 0.6
    i_gain = 0.1
    d_gain = -0.1
    target_val = 960 
    accumulated_error = 710
    last_error = 710
    last_time = 0

    for t,e in  [(0.5, 600), (1.0,  780), (1.5, 912), (2.0, 1100), (2.5, 1500), (3.0, 1300), (3.5, 1102), (4.0, 924), (4.5, 882), (5.0, 956), (5.5, 1025), (6.0, 998), (6.5, 950), (7.0, 956), (7.5, 968)] :
        print(e)
        output, accumulated_error, last_error = pid_control(
            p_gain, i_gain, d_gain, target_val, e, accumulated_error, last_error, t-last_time
        )
        last_time = t
        print(f"At time {t}, control output is {output}")
test_pid_control()