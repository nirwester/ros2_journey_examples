# Instructions: modify the Inputs parameters in the script how you like,
# then execute it with the command # "python3 joint_control_with_delay.py".
# This will display a graph showing the time-evolution of the position
# and the velocity of the joint.

import numpy as np
import matplotlib.pyplot as plt


# Inputs
k_kp = 5.0
k_dt_actuation = 0.05
k_dt_feedback = 0.05
k_control_period = 0.01  # 20Hz
k_acc_lim = 0.5
k_target_position = 0.05
k_plot_time_limit = 1.5
k_use_prediction = True


def update_vel(current_vel, command, acc_lim, control_period):
    if (command >= current_vel):
        updated_vel = current_vel + acc_lim * control_period
        updated_vel = min(updated_vel, command)
    else:
        updated_vel = current_vel - acc_lim * control_period
        updated_vel = max(updated_vel, command)
    return updated_vel


def update_position(current_position, current_vel, next_vel, control_period):
    average_vel = (current_vel + next_vel) / 2.0
    return current_position + average_vel * control_period


def compute_command(distance):
    return k_kp * distance


def compute_command_with_prediction(distance, velocity):
    error = distance - velocity * (k_dt_actuation + k_dt_feedback)
    return k_kp * error


def get_old_element(history, delayed_time, control_period):
    if (control_period <= 0.0):
        raise("Control period can't null or negative")
    index_delta = int(
        delayed_time / control_period) if (delayed_time > 0.0) else 0
    if (len(history) < index_delta + 1):
        return 0.0
    return history[len(history) - index_delta - 1]


if __name__ == "__main__":
    position = 0.0
    velocity = 0.0
    time = 0.0
    cmd_history = [0.0]
    time_history = [0.0]
    pos_history = [0.0]
    vel_history = [0.0]
    while (time < k_plot_time_limit):
        time = time + k_control_period
        time_history.append(time)
        read_position = get_old_element(
            pos_history, k_dt_feedback, k_control_period)
        error = k_target_position - read_position
        if (k_use_prediction):
            read_velocity = get_old_element(
                vel_history, k_dt_feedback, k_control_period)
            cmd_history.append(
                compute_command_with_prediction(error, read_velocity))
        else:
            cmd_history.append(compute_command(error))
        applied_cmd = get_old_element(
            cmd_history, k_dt_actuation, k_control_period)
        next_vel = update_vel(velocity, applied_cmd,
                              k_acc_lim, k_control_period)
        position = update_position(
            position, velocity, next_vel, k_control_period)
        print("POs: ", position, " Read pos: ", read_position)
        velocity = next_vel
        pos_history.append(position)
        vel_history.append(next_vel)

    time_npy = np.array(time_history)
    pos_npy = np.array(pos_history)
    vel_npy = np.array(vel_history)
    plt.plot(time_npy, pos_npy)
    plt.plot(time_npy, vel_npy, 'r-')
    plt.grid()
    plt.xlabel("Time [s]")
    plt.ylabel("Position (blue) and velocity (red)")
    plt.show()
