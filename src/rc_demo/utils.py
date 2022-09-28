import rospy
import math

from typing import Tuple, List

from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class ControlMessage:
    """Send control message via MavROS"""

    def __init__(self) -> None:
        self.control_pub = rospy.Publisher('/rc_demo', String, queue_size=1)

    def send_control(self, roll: float, pitch: float, yaw: float, throttle: float) -> None:
        """Send control message

        Args:
            roll: roll
            pitch: pitch
            yaw: yaw
            throttle: throttle
        """
        msg = String()
        msg.data = "{roll},{pitch},{yaw},{throttle}".format(roll=roll,pitch=pitch,yaw=yaw,throttle=throttle)
        self.control_pub.publish(msg)


class WaypointMessage:
    """Send waypoint message via MavROS"""

    def __init__(self) -> None:
        self.waypoint_pub = rospy.Publisher('/waypoint_generator/waypoints',
                                            Path,
                                            queue_size=1)
        self.odom_sub = rospy.Subscriber('/Odometry',
                                         Odometry,
                                         self.callback_odometry)
        self._timer_send_waypoint = rospy.Timer(rospy.Duration(3.0),
                                                self._loop_send_waypoint)
        self.distance_tolerance = 0.5

        self.current_position = Odometry()
        self.target_position = PoseStamped()
        self.is_moving = False
        self.waypoints = []
        self.send_seq = 0


    def clear_waypoint(self) -> None:
        self.waypoints.clear()
        self.target_position.pose = self.current_position.pose.pose
        self.is_moving = False


    def add_waypoint(self, x: float, y: float, z: float) -> None:
        self.waypoints.append((x, y, z))


    def add_waypoints(self, points: List[Tuple[float, float, float]]) -> None:
        for point in points:
            self.add_waypoint(*point)


    def _check_current_location(self) -> bool:
        if len(self.waypoints) < 1:
            return False

        target_point = self.target_position.pose.position
        current_point = self.current_position.pose.pose.position

        distance = math.sqrt(math.pow(target_point.x - current_point.x, 2) + math.pow(target_point.y - current_point.y, 2) + math.pow(target_point.z - current_point.z, 2))

        if distance < self.distance_tolerance:
            return True

        return False

    def _loop_send_waypoint(self, event = None) -> None:
        target_point = None
        if self.is_moving and not self._check_current_location():
            target_point = self.target_position
        else:
            self.is_moving = False

        if len(self.waypoints) < 1:
            target_point = self.target_position

        self.send_seq += 1

        if target_point == None:
            target_point = self.waypoints.pop(0)

            pose = PoseStamped()
            pose.header.seq = self.send_seq
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = target_point[0]
            pose.pose.position.y = target_point[1]
            pose.pose.position.z = target_point[2]
            pose.pose.orientation.w = 1.0
        else:
            pose = target_point

        msg = Path()
        msg.header.seq = self.send_seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.poses.append(pose)
        self.target_position = pose

        self.waypoint_pub.publish(msg)
        self.is_moving = True

    def callback_odometry(self, msg: Odometry) -> None:
        self.current_position = msg

        if self.target_position.pose.position.x == 0 and self.target_position.pose.position.y == 0 and self.target_position.pose.position.z == 0:
            self.target_position.pose = msg.pose.pose


class MAVROSCommander:
    def __init__(self):
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    def set_mode(self, mode):
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = mode
        try:
            resp1 = self.set_mode_client(0, offb_set_mode.custom_mode)
            return resp1.mode_sent
        except:
            return False

    def set_arm(self, value):
        arm_cmd = CommandBool()
        arm_cmd.value = value
        try:
            arm_client_1 = self.arming_client(arm_cmd.value)
            return arm_client_1.success
        except:
            return False

