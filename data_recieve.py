import socket
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Lists to hold the time series data
time_series = []
left_hip_data = []
left_knee_data = []
left_wheel_data = []
right_hip_data = []
right_knee_data = []
right_wheel_data = []

# Lists to hold the deviation data
left_hip_deviation = []
left_knee_deviation = []
left_wheel_deviation = []
right_hip_deviation = []
right_knee_deviation = []
right_wheel_deviation = []

def udp_server(sock):
    print(f"Listening on {server_ip}:{server_port}")
    
    start_time = time.time()

    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        (left_hip_iq, left_knee_iq, left_wheel_iq,
         right_hip_iq, right_knee_iq, right_wheel_iq) = struct.unpack('ffffff', data)
        
        current_time = time.time() - start_time
        time_series.append(current_time)
        
        # Append the actual data
        left_hip_data.append(left_hip_iq)
        left_knee_data.append(left_knee_iq)
        left_wheel_data.append(left_wheel_iq)
        right_hip_data.append(right_hip_iq)
        right_knee_data.append(right_knee_iq)
        right_wheel_data.append(right_wheel_iq)
        
        # Calculate the deviations from the previous value
        if len(time_series) > 1:
            left_hip_deviation.append(left_hip_iq - left_hip_data[-2])
            left_knee_deviation.append(left_knee_iq - left_knee_data[-2])
            left_wheel_deviation.append(left_wheel_iq - left_wheel_data[-2])
            right_hip_deviation.append(right_hip_iq - right_hip_data[-2])
            right_knee_deviation.append(right_knee_iq - right_knee_data[-2])
            right_wheel_deviation.append(right_wheel_iq - right_wheel_data[-2])
        else:
            # If this is the first data point, there is no previous data to compare with
            left_hip_deviation.append(0)
            left_knee_deviation.append(0)
            left_wheel_deviation.append(0)
            right_hip_deviation.append(0)
            right_knee_deviation.append(0)
            right_wheel_deviation.append(0)
        
        print(f"Received data from {addr}:")
        print(f"  Left Hip IQ: {left_hip_iq}, Left Knee IQ: {left_knee_iq}, Left Wheel IQ: {left_wheel_iq}")
        print(f"  Right Hip IQ: {right_hip_iq}, Right Knee IQ: {right_knee_iq}, Right Wheel IQ: {right_wheel_iq}")

def update_plot(i, time_series, left_hip_deviation, left_knee_deviation, left_wheel_deviation,
                right_hip_deviation, right_knee_deviation, right_wheel_deviation, deviation_lines):
    # Update line data for each deviation value
    deviation_lines[0].set_data(time_series, left_hip_deviation)
    deviation_lines[1].set_data(time_series, left_knee_deviation)
    deviation_lines[2].set_data(time_series, left_wheel_deviation)
    deviation_lines[3].set_data(time_series, right_hip_deviation)
    deviation_lines[4].set_data(time_series, right_knee_deviation)
    deviation_lines[5].set_data(time_series, right_wheel_deviation)

    # Adjust plot limits
    ax.set_xlim(min(time_series), max(time_series))
    ax.relim()
    ax.autoscale_view()

    return deviation_lines

if __name__ == '__main__':
    # Set up the UDP server
    server_ip = '127.0.0.1'  # IP to listen on
    server_port = 5000       # Port to listen on

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((server_ip, server_port))

    # Set up the plot
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)  # Initial limit, will be adjusted dynamically
    ax.set_ylim(-10, 10)  # Adjust based on expected data range
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Deviation in Motor IQ Values')

    # Lines for the deviations
    line_left_hip, = ax.plot([], [], label='Left Hip Deviation', color='blue')
    line_left_knee, = ax.plot([], [], label='Left Knee Deviation', color='green')
    line_left_wheel, = ax.plot([], [], label='Left Wheel Deviation', color='cyan')
    line_right_hip, = ax.plot([], [], label='Right Hip Deviation', color='red')
    line_right_knee, = ax.plot([], [], label='Right Knee Deviation', color='magenta')
    line_right_wheel, = ax.plot([], [], label='Right Wheel Deviation', color='yellow')
    deviation_lines = [line_left_hip, line_left_knee, line_left_wheel,
                       line_right_hip, line_right_knee, line_right_wheel]

    plt.legend()

    # Start the UDP server in a separate thread
    import threading
    udp_thread = threading.Thread(target=udp_server, args=(sock,))
    udp_thread.daemon = True
    udp_thread.start()

    # Start the animation to update the plot
    ani = animation.FuncAnimation(fig, update_plot, fargs=(time_series, left_hip_deviation, left_knee_deviation, left_wheel_deviation,
                                                           right_hip_deviation, right_knee_deviation, right_wheel_deviation, deviation_lines),
                                  interval=100, blit=True)

    # Show the plot
    plt.show()
