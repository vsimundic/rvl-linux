#!/usr/bin/env python3

import rospy
from xela_server_ros.msg import SensStream
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# Configuration
buffer_size = 200  # how many points to show per line
num_taxels = 16
topic_name = "/xServTopic"

# Data buffer
fz_buffers = [deque(maxlen=buffer_size) for _ in range(num_taxels)]
time_buffer = deque(maxlen=buffer_size)

def callback(msg):
    global fz_buffers, time_buffer
    if not msg.sensors:
        return

    now = rospy.get_time()
    time_buffer.append(now)

    for i, force in enumerate(msg.sensors[0].forces[:num_taxels]):
        fz_buffers[i].append(force.z)

def main():
    rospy.init_node("xela_fz_live_plot")
    rospy.Subscriber(topic_name, SensStream, callback)

    plt.ion()
    fig, ax = plt.subplots()
    lines = [ax.plot([], [], label=f"fz{i}")[0] for i in range(num_taxels)]

    ax.set_ylim(-10, 10)
    ax.set_title("Live fz Forces from Xela Sensor")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force Z (N)")
    ax.legend(loc="upper right")

    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():
        if len(time_buffer) < 2:
            rate.sleep()
            continue

        t_vals = np.array(time_buffer)
        t_vals = t_vals - t_vals[0]  # make time start at 0

        for i in range(num_taxels):
            lines[i].set_data(t_vals, fz_buffers[i])

        ax.set_xlim(t_vals[0], t_vals[-1])
        ax.relim()
        ax.autoscale_view(scaley=True)
        plt.pause(0.01)
        rate.sleep()

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()