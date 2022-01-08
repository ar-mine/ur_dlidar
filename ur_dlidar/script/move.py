import sys
from math import dist, fabs, cos
from threading import Thread

# ROS
import rospy
from std_msgs.msg import Float32
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetSpeedSliderFraction

# moveit
import moveit_commander
from moveit_commander.conversions import pose_to_list, list_to_pose
from helper_func import *
from moveit_msgs.msg import CollisionObject, RobotTrajectory
from moveit_msgs.srv import GetStateValidityRequest


def all_close(goal, actual, tolerance=0.05):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroup(Thread):
    def __init__(self):
        super(MoveGroup, self).__init__()

        # Moveit robot initialize
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot_cmd = moveit_commander.RobotCommander()
        self.plan_scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group_cmd = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot_state = self.move_group_cmd.get_current_state()

        self.planning_frame = self.move_group_cmd.get_planning_frame()
        self.eef_link = self.move_group_cmd.get_end_effector_link()
        self.group_names = self.robot_cmd.get_group_names()

        # Set topic listener or service
        # rospy.Subscriber("move_group/speed_scaling", Float32, self._speed_scaling, queue_size=1)
        self.speed_srv = rospy.ServiceProxy(
            "/ur_hardware_interface/set_speed_slider", SetSpeedSliderFraction
        )

        # Updated parameters
        self.change_flag = False
        # True for finished, False for during process
        self.execute_flag = True
        # joint_states -> list(order):
        # 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        self.joint_states = self.move_group_cmd.get_current_joint_values()
        self.joint_goals = self.joint_states
        self.pose_states = self.move_group_cmd.get_current_pose().pose
        self.pose_goals = self.pose_states
        self.plan = RobotTrajectory()
        # 0 for no goal, 1 for joints goal, 2 for pose goal
        self.goal_type = 0
        # 0 for no path, 1 for normal planning, 2 for cartesian planning
        self.path_type = 0

        # Thread flag
        self.done = False

        # Delay 2s to keep the initialization above stable
        rospy.sleep(2)

    def run(self):
        while not self.done:
            if not self.goal_type:
                rospy.sleep(0.1)
            else:
                self.move_group_cmd.execute(self.plan, wait=False)
                while not self.done and not self._update_and_check():
                    if self.change_flag:
                        self.change_flag = False
                        self.move_group_cmd.execute(self.plan, wait=False)
                    rospy.sleep(0.05)
                self.clear_targets()

    def stop(self):
        self.done = True
        self.move_group_cmd.stop()

    def _update_and_check(self):
        """ Update joints and pose, and check if it should go new states
            :return(bool)
                0 for no change, 1 for change happens
        """
        self.joint_states = self.move_group_cmd.get_current_joint_values()
        self.pose_states = self.move_group_cmd.get_current_pose().pose

        return self._all_close()

    def _all_close(self, tolerance=0.01):
        ret = False
        if self.goal_type in [0, 1]:
            ret = all_close(goal=self.joint_goals, actual=self.joint_states, tolerance=tolerance)
        if self.goal_type in [0, 2]:
            ret = all_close(goal=self.pose_goals, actual=self.pose_states, tolerance=tolerance)
        return ret

    def set_targets(self, goal):
        rospy.loginfo("Begin set target")
        ret, self.plan, planning_time, error_code = self.move_group_cmd.plan(goal)
        if type(goal) is list:
            self.joint_goals = goal
            self.goal_type = 1
        elif type(goal) is Pose:
            self.pose_goals = goal
            self.goal_type = 2
        self.path_type = 1
        self.execute_flag = False
        rospy.loginfo("Set target successful")

    def set_cartesian_targets(self, goal: Pose):
        rospy.loginfo("Begin set target")
        (self.plan, fraction) = self.move_group_cmd.compute_cartesian_path(
            [goal],  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        self.pose_goals = goal
        self.goal_type = 2
        self.path_type = 2
        self.execute_flag = False
        rospy.loginfo("Set target successful")

    def _scale_cartesian_path(self, goal):
        (plan, fraction) = self.move_group_cmd.compute_cartesian_path(
                                                                    [goal],  # waypoints to follow
                                                                    0.01,  # eef_step
                                                                    0.0)  # jump_threshold
        self.plan = self.move_group_cmd.retime_trajectory(self.move_group_cmd.get_current_state(),
                                                          plan,
                                                          velocity_scaling_factor=self.speed_scaling,
                                                          acceleration_scaling_factor=self.speed_scaling,
                                                          algorithm="time_optimal_trajectory_generation")

    def clear_targets(self):
        rospy.loginfo("Begin clear targets")
        self.move_group_cmd.stop()
        self.joint_goals = self.joint_states
        self.pose_goals = self.pose_states
        self.move_group_cmd.clear_pose_targets()
        self.goal_type = 0
        self.path_type = 0
        self.execute_flag = True
        rospy.loginfo("Clear targets successful")

    def wait_cb(self, callback=None):
        rospy.loginfo("Waiting")
        while not self.execute_flag and not self.done:
            if callback is not None:
                callback()
        rospy.sleep(0.5)
        rospy.loginfo("Waiting finished")

    def stop_c(self):
        self.move_group_cmd.stop()

    def resume(self):
        if self.path_type == 2:
            (self.plan, fraction) = self.move_group_cmd.compute_cartesian_path(
                [self.pose_goals],  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold
            self.change_flag = True
