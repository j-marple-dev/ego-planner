#!/usr/bin/env python3

import sys
import rospy
import yaml
import time
import threading

from mavros_msgs.msg import RCIn

from utils import ControlMessage, WaypointMessage

waypoint_thread_lock = threading.Lock()

class RCHandler:
    """Receieve RC stick data."""

    def __init__(self,
                 use_rc_control: bool = True,
                 waypoint_path: str = "",
                 target_waypoint: str = "waypoint_0") -> None:
        self.use_rc_control = use_rc_control
        self.target_waypoint = target_waypoint

        self.rc_sub = rospy.Subscriber("/mavros/rc/in", RCIn, self.callback_rc_in)
        self.control_msg = ControlMessage()
        self.waypoint_msg = WaypointMessage()

        self.is_waypoint_working = False

        self.is_calibrated = False
        self.calib_count = 0

        self.val_range = [9999, 0.0]

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0

        self.cmd_magnitude = 1.0

        if waypoint_path != "":
            with open(waypoint_path, 'r') as f:
                self.waypoints = yaml.load(f, yaml.SafeLoader)
        else:
            self.waypoints = {
                'waypoint_0': [[0.0, 0.0, 1.2]],
                'waypoint_1': [[0.0, 0.0, 1.2]],
            }

    def _start_waypoint(self) -> None:
        if self.is_waypoint_working:
            return

        waypoint_thread_lock.acquire()

        self.is_waypoint_working = True
        self.waypoint_msg.clear_waypoint()
        self.waypoint_msg.add_waypoints(self.waypoints[self.target_waypoint])

        waypoint_thread_lock.release()

    def _terminate_waypoint(self) -> None:
        waypoint_thread_lock.acquire()

        self.is_waypoint_working = False
        self.waypoint_msg.clear_waypoint()

        waypoint_thread_lock.release()

    def callback_rc_in(self, msg: RCIn) -> None:
        """Receieves /mavros/rc/in

        Receieved stick message will be sent
        """

        # Check waypoint switch
        if 1300 < msg.channels[6] < 1700:
            self._start_waypoint()
        elif 1000 < msg.channels[6] < 1300:
            self._terminate_waypoint()

        for i in range(4):
            self.val_range[0] = min(self.val_range[0], msg.channels[i])
            self.val_range[1] = max(self.val_range[1], msg.channels[i])

        sub_val = self.val_range[0]
        divider = (self.val_range[1] - self.val_range[0])

        if divider < 500:
            return

        # Normalize 0 ~ 1
        self.roll = (msg.channels[0] - sub_val) / divider
        self.pitch = ((msg.channels[1] - sub_val) / divider)
        self.throttle = (msg.channels[2] - sub_val) / divider
        self.yaw = (msg.channels[3] - sub_val) / divider

        # Normalize -1 ~ 1
        self.roll = (self.roll - 0.5) * 2
        self.pitch = (self.pitch - 0.5) * 2
        self.throttle = (self.throttle - 0.5) * 2
        self.yaw = (self.yaw - 0.5) * 2

        # Need to calibrate for the RC
        # TODO(jeikeilim): Find better way to calibrate RC
        if self.roll == 0.0 and self.pitch == 0.0 and not self.is_calibrated:
            self.calib_count += 1

            if self.calib_count > 10:
                self.is_calibrated = True
        else:
            self.calib_count = 0

        if not self.is_calibrated or not self.use_rc_control:
            return

        self.control_msg.send_control(-self.roll * self.cmd_magnitude,
                          -self.pitch * self.cmd_magnitude,
                          -self.yaw * self.cmd_magnitude,
                          self.throttle * self.cmd_magnitude)


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    rospy.init_node("flight_control_node", anonymous=True)

    use_rc_control = rospy.get_param('~rc_control', False)
    waypoint_path = rospy.get_param('~waypoint', "")
    target_waypoint = rospy.get_param('~target_waypoint', "waypoint_0")

    rc_handler = RCHandler(use_rc_control, waypoint_path, target_waypoint)

    rospy.spin()

