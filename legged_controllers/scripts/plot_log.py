import numpy as np
import matplotlib.pyplot as plt
import os

def read_csv_numpy(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1, skip_footer=1)
    return data

def plot_data(data, indices, column_names):
    times = data[:, 0]  # get time stamp
    times -= times[0]  # shift to zero

    assert len(indices) == len(column_names)

    plt.figure(figsize=(10, 6))
    for i, index in enumerate(indices):
        plt.subplot(len(indices), 1, i + 1)
        plt.plot(times, data[:, index], label=column_names[i])
        # plt.xlabel('Time (s)')
        plt.ylabel(column_names[i])
        # plt.title(f'Time Series Plot for {column_names[i]}')
        plt.legend()

    plt.tight_layout()
    plt.show()

def main():
    log_dir = os.path.join(os.path.expanduser('~'), 'mpc_log')
    file_name = 'output_2024-07-11_14-22-03.csv'
    file_path = os.path.join(log_dir, file_name)

    # indices = [7, 8, 9, 10, 11, 12]
    # column_names = ['x', 'y', 'z', 'theta_z', 'theta_y', 'theta_x']

    # indices = [1,2,3,4,5,6]
    # column_names = ['v_x', 'y', 'z', 'theta_z', 'theta_y', 'theta_x']

    # indices = [13,14,15,16,17,18]
    # column_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    # indices = [25,26,27,28,29,30]
    # column_names = ['v_l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    indices = [37,38,39,40,41,42]
    column_names = ['error_l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    data = read_csv_numpy(file_path)

    selected_start_time = 0
    selected_end_time = 15
    frequency = 400
    selected_data = data[selected_start_time*frequency:selected_end_time*frequency, :]
    selected_data[:, 37:43] = selected_data[:, 13:19] - selected_data[:, 37:43]

    plot_data(selected_data, indices, column_names)

if __name__ == '__main__':
    main()