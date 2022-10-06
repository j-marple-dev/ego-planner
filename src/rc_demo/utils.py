import rospy
import math

from typing import Tuple, List

from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker


def compute_distance(x: Pose, y: Pose) -> float:
    return math.sqrt(math.pow(x.x - y.x, 2) + math.pow(x.y - y.y, 2) + math.pow(x.z - y.z, 2))


def get_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert an Euler angle to a quaternion.

    Args:
        roll: The roll (rotation around x-axis) angle in radians.
        pitch: The pitch (rotation around y-axis) angle in radians.
        yaw: The yaw (rotation around z-axis) angle in radians.

    Return:
        qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return (qx, qy, qz, qw)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert a quaternion to Euler angle.

    Args:
        x: quaternion x
        y: quaternion y
        z: quaternion z
        w: quaternion w

    Return:
        roll, pitch, yaw in radians.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)


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

    def __init__(self, distance_tolerance: float = 0.5) -> None:
        self.waypoint_pub = rospy.Publisher('/waypoint_generator/waypoints',
                                            Path,
                                            queue_size=1)
        self.odom_sub = rospy.Subscriber('/Odometry',
                                         Odometry,
                                         self.callback_odometry)
        self.goal_point_sub = rospy.Subscriber('/ego_planner_node/goal_point',
                                               Marker,
                                               self.callback_goal_point)
        self._timer_send_waypoint = rospy.Timer(rospy.Duration(3.0),
                                                self._loop_send_waypoint)
        self.distance_tolerance = distance_tolerance

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

    def add_waypoint_yaw(self, x: float, y: float, z: float, yaw: float) -> None:
        self.waypoints.append((x, y, z, yaw))

    def add_waypoints(self, points: List[Tuple[float, float, float]]) -> None:
        for point in points:
            if len(point) > 3:
                self.add_waypoint_yaw(*point)
            else:
                self.add_waypoint(*point)

    def _check_current_location(self) -> bool:
        if len(self.waypoints) < 1:
            return False

        target_point = self.target_position.pose.position
        current_point = self.current_position.pose.pose.position

        distance = compute_distance(target_point, current_point)

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
            pose.pose.orientation = self.current_position.pose.pose.orientation

            if len(target_point) > 3:
                quaternion = get_quaternion_from_euler(0, 0, math.radians(-target_point[3]+90))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
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

    def callback_goal_point(self, msg: Marker) -> None:
        if msg.id != 1:
            return

        if compute_distance(msg.pose.position, self.target_position.pose.position) < self.distance_tolerance:
            self.target_position.pose.position = msg.pose.position


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

