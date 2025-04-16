#!/usr/bin/env python3

import rospy
from xela_server_ros.msg import SensStream
import os
from datetime import datetime
import time

class XelaFlatLogger:
    def __init__(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path_param = rospy.get_param("~output_path", f"/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_force_open-20250402/tactile_data/fz_only_{timestamp}.csv")
        self.file_path = os.path.expanduser(path_param)
        self.rate_hz = rospy.get_param("~rate", 50)  # Rate in Hz
        self.min_dt = 1.0 / self.rate_hz
        self.last_log_time = 0

        self.file = open(self.file_path, "w")

        # Write header: fz0 to fz15
        header = ["timestamp"] + [f"fz{i}" for i in range(16)]
        self.file.write(",".join(header) + "\n")

        rospy.loginfo(f"[XelaFlatLogger] Logging Fz components only to: {self.file_path}")
        rospy.loginfo(f"[XelaFlatLogger] Rate limiting to {self.rate_hz} Hz")
        rospy.Subscriber("/xServTopic", SensStream, self.callback)

    def callback(self, msg):
        now = time.time()
        # Optional rate limiting (uncomment if needed):
        if now - self.last_log_time < self.min_dt:
            return
        self.last_log_time = now

        for sensor in msg.sensors:
            fz_values = [f"{force.z}" for force in sensor.forces]
            self.file.write(f"{now},")
            self.file.write(",".join(fz_values) + "\n")

    def shutdown_hook(self):
        rospy.loginfo("[XelaFlatLogger] Shutting down, closing file.")
        self.file.close()

if __name__ == '__main__':
    rospy.init_node('xela_flat_logger', anonymous=True)
    logger = XelaFlatLogger()
    rospy.on_shutdown(logger.shutdown_hook)
    rospy.spin()