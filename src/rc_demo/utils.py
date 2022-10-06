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
    sin_roll = math.sin(roll/2)
    cos_roll = math.cos(roll/2)

    sin_pitch = math.sin(pitch/2)
    cos_pitch = math.cos(pitch/2)

    sin_yaw = math.sin(yaw/2)
    cos_yaw = math.cos(yaw/2)


    qx = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    qy = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    qz = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
    qw = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw

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

    def __init__(
        self,
        distance_tolerance: float = 0.5,
        angle_tolerance: float = 10.0,
        check_yaw: bool = True,
    ) -> None:
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
        self.check_yaw = check_yaw
        self.angle_tolerance = angle_tolerance

        self.current_position = Odometry()
        """Target position is the designated waypoint"""
        self.target_position = PoseStamped()
        self.target_position.pose.orientation.w = 1.0
        """Goal position is modified waypoint from ego-planner"""
        self.goal_position = Pose()
        self.goal_position.orientation.w = 1.0
        self.is_moving = False
        self.waypoints = []
        self.send_seq = 0

        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.offset_Y = 0.0

    def set_offset(self, x: float, y: float, z: float, Y: float) -> None:
        self.offset_x = x
        self.offset_y = y
        self.offset_z = z
        self.offset_Y = Y

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

        # goal_point = self.target_position.pose
        goal_point = self.goal_position
        current_point = self.current_position.pose.pose

        distance = compute_distance(goal_point.position, current_point.position)

        rpy_goal = quaternion_to_euler(goal_point.orientation.x,
                                       goal_point.orientation.y,
                                       goal_point.orientation.w,
                                       goal_point.orientation.z)
        rpy_current = quaternion_to_euler(current_point.orientation.x,
                                          current_point.orientation.y,
                                          current_point.orientation.w,
                                          current_point.orientation.z)
        yaw_diff = min((2 * math.pi) - abs(rpy_goal[2]-rpy_current[2]), abs(rpy_goal[2]-rpy_current[2]))
        yaw_diff = math.degrees(yaw_diff)

        # Debug logging
        if False:
            rospy.loginfo(f"[Distance to the goal] {distance:.3f}m, [Angle difference]: {yaw_diff:.3f} degree")
            rospy.loginfo(f"[   GOAL POSE] x: {goal_point.position.x:8.3f}, y: {goal_point.position.y:8.3f}, z: {goal_point.position.z:8.3f}, R: {rpy_goal[0]:8.3f}, P: {rpy_goal[1]:8.3f}, Y: {rpy_goal[2]:8.3f}")
            rospy.loginfo(f"[CURRENT POSE] x: {current_point.position.x:8.3f}, y: {current_point.position.y:8.3f}, z: {current_point.position.z:8.3f}, R: {rpy_current[0]:8.3f}, P: {rpy_current[1]:8.3f}, Y: {rpy_current[2]:8.3f}")

        if self.check_yaw:
            if distance < self.distance_tolerance and yaw_diff < self.angle_tolerance:
                return True
        else:
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

            # apply offset
            temp_x = target_point[0] - self.offset_x
            temp_y = target_point[1] - self.offset_y
            temp_rad = self.offset_Y / 180.0 * math.pi

            pose = PoseStamped()
            pose.header.seq = self.send_seq
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = temp_x * math.cos(temp_rad) - temp_y * math.sin(temp_rad)
            pose.pose.position.y = temp_x * math.sin(temp_rad) + temp_y * math.cos(temp_rad)
            pose.pose.position.z = target_point[2] - self.offset_z
            if self.current_position.pose.pose.orientation.x == 0 and \
               self.current_position.pose.pose.orientation.y == 0 and \
               self.current_position.pose.pose.orientation.z == 0 and \
               self.current_position.pose.pose.orientation.w == 0:
                pose.pose.orientation.w = 1.0
            else:
                pose.pose.orientation.x = self.current_position.pose.pose.orientation.x
                pose.pose.orientation.y = self.current_position.pose.pose.orientation.y
                pose.pose.orientation.z = self.current_position.pose.pose.orientation.z
                pose.pose.orientation.w = self.current_position.pose.pose.orientation.w

            if len(target_point) > 3:
                quaternion = get_quaternion_from_euler(0, 0, math.radians(-(target_point[3] - self.offset_Y)+90))
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
        self.goal_position.position = pose.pose.position
        self.goal_position.orientation = pose.pose.orientation

        # Debugging log
        if False:
            rospy.loginfo(f"[CURRENT  WAYPOINT]: \n{pose.pose.position}")
            rospy.loginfo(f"[   ORIENTATION   ]: \n{pose.pose.orientation}")

        self.waypoint_pub.publish(msg)
        self.is_moving = True

    def callback_odometry(self, msg: Odometry) -> None:
        self.current_position = msg

        if self.target_position.pose.position.x == 0 and self.target_position.pose.position.y == 0 and self.target_position.pose.position.z == 0:
            self.goal_position.position = msg.pose.pose.position
            self.goal_position.orientation = msg.pose.pose.orientation
            self.target_position.pose.position = msg.pose.pose.position
            self.target_position.pose.orientation = msg.pose.pose.orientation

    def callback_goal_point(self, msg: Marker) -> None:
        if msg.id != 1:
            return

        if compute_distance(msg.pose.position, self.target_position.pose.position) < (self.distance_tolerance * 2):
            # self.target_position.pose.position = msg.pose.position
            self.goal_position.position = msg.pose.position


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

