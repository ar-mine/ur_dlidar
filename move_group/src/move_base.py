import sys
from math import dist, fabs, cos
from threading import Thread
from scipy.interpolate import UnivariateSpline, interp1d
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import threading

# ROS
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetSpeedSliderFraction
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllersRequest, ListControllers
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# moveit
import moveit_commander
from moveit_commander.conversions import pose_to_list, list_to_pose
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
        self.speed_srv = rospy.ServiceProxy(
            "/ur_hardware_interface/set_speed_slider", SetSpeedSliderFraction
        )
        # ROS service for switching controllers
        self.switch_controller_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        # ROS service for loading controllers
        self.load_controller_srv = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController
        )
        # ROS service for listing controllers
        self.list_controller_srv = rospy.ServiceProxy(
            "controller_manager/list_controllers", ListControllers
        )
        self._srv_init()

        # Controller client
        self.controller_client = None

        # Updated parameters
        self.speed_scaling = 1
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
        # 0 for no group, 1 for pos group, 2 for vel group
        self.mode = 0

        # eef parameters
        self.eef_speed = None
        self.eef_effort = None
        self.eef_x_pub = rospy.Publisher("/speed/eef_x", Float64, queue_size=1)
        self.eef_y_pub = rospy.Publisher("/speed/eef_y", Float64, queue_size=1)
        self.eef_z_pub = rospy.Publisher("/speed/eef_z", Float64, queue_size=1)
        self.eef_pub = rospy.Publisher("/speed/marker", MarkerArray, queue_size=1)

        # Thread flag
        self.done = False
        # Lock
        self.lock = threading.RLock()

        # Delay 2s to keep the initialization above stable
        rospy.sleep(2)

    def run(self):
        while not self.done:
            if not self.goal_type:
                rospy.sleep(0.1)
            else:
                if self.mode:
                    self._client_execute()
                else:
                    self._moveit_execute()

    def stop(self):
        self.done = True
        self.move_group_cmd.stop()

    def set_targets(self, goal):
        rospy.logdebug("Begin set target")
        ret, self.plan, planning_time, error_code = self.move_group_cmd.plan(goal)
        if type(goal) is list:
            self.joint_goals = goal
            self.goal_type = 1
        elif type(goal) is Pose:
            self.pose_goals = goal
            self.goal_type = 2
        self.path_type = 1
        self.execute_flag = False
        rospy.logdebug("Set target successful")

    def set_cartesian_targets(self, goal: Pose):
        rospy.logdebug("Begin set target")
        (self.plan, fraction) = self.move_group_cmd.compute_cartesian_path(
            [goal],  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        self._traj_refine(rate=50)
        self.pose_goals = goal
        self.goal_type = 2
        self.path_type = 2
        self.execute_flag = False
        rospy.logdebug("Set target successful")

    def clear_targets(self):
        rospy.logdebug("Begin clear targets")
        self.move_group_cmd.stop()
        self.joint_goals = self.joint_states
        self.pose_goals = self.pose_states
        self.move_group_cmd.clear_pose_targets()
        self.goal_type = 0
        self.path_type = 0
        self.execute_flag = True
        rospy.logdebug("Clear targets successful")

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

    def switch_controller(self, controller: str):
        ret = True
        controller_list = self.list_controller_srv.call().controller
        c_name = [c.name for c in controller_list]
        c_state = [c.state for c in controller_list]
        if controller not in c_name:
            rospy.logerr("The controller {} doesn't exist.".format(controller))
            ret = False
        else:
            c_idx = c_name.index(controller)
            if c_state[c_idx] == "running":
                rospy.loginfo("The controller {} is running, no need to switch.".format(controller))
            else:
                if not c_state[c_idx] == "initialized":
                    srv = LoadControllerRequest()
                    srv.name = controller
                    self.load_controller_srv(srv)

                c_name.remove("joint_state_controller")
                c_name.remove(controller)
                srv = SwitchControllerRequest()
                srv.stop_controllers = c_name
                srv.start_controllers = [controller]
                srv.strictness = SwitchControllerRequest.BEST_EFFORT
                self.switch_controller_srv.call(srv)
                rospy.loginfo("The controller {} is switched successfully.".format(controller))

            if controller == "joint_group_pos_controller":
                self.controller_client = rospy.Publisher("/joint_group_pos_controller/command",
                                                         Float64MultiArray,
                                                         queue_size=1)
                self.mode = 1
            elif controller == "joint_group_vel_controller":
                self.controller_client = rospy.Publisher("/joint_group_vel_controller/command",
                                                         Float64MultiArray,
                                                         queue_size=1)
                self.mode = 2
            # elif controller == "scaled_pos_joint_traj_controller":
            #     self.controller_client = actionlib.SimpleActionClient(
            #                                 "{}/follow_joint_trajectory".format(self.controller_client),
            #                                 FollowJointTrajectoryAction)
            else:
                self.controller_client = actionlib.SimpleActionClient(
                                            "{}/follow_joint_trajectory".format(controller),
                                            FollowJointTrajectoryAction)

        return ret

    # todo
    def display_trajectory(self, plan):
        pass

    # private:
    def _traj_refine(self, rate=50):
        speed_scaling = 0.5

        plan = self.plan.joint_trajectory.points
        positions = np.array([p.positions for p in plan])
        time = np.array([p.time_from_start.to_sec() for p in plan])
        duration = (time[-1] - time[0])/speed_scaling
        point_sum = int(duration * rate) + 1
        time_factor = duration / (point_sum - 1)
        new_time = np.linspace(time[0], time[-1], num=point_sum, endpoint=True)
        new_positions = []
        for i in range(6):
            f_spline = interp1d(time, positions[:, i])
            new_positions.append(f_spline(new_time))
        new_positions = np.array(new_positions)
        new_time /= speed_scaling
        plan = RobotTrajectory()
        for i in range(len(new_time)):
            if i == 0:
                point = JointTrajectoryPoint()
                point.positions = new_positions[:, 0]
                point.velocities = np.zeros(6, )
                point.time_from_start = rospy.Duration(0)
            point = JointTrajectoryPoint()
            point.positions = new_positions[:, i]
            if i == len(new_time) - 1:
                point.velocities = np.zeros(6, )
            else:
                point.velocities = (new_positions[:, i + 1] - new_positions[:, i]) / time_factor
            point.time_from_start = rospy.Duration(new_time[i])
            plan.joint_trajectory.points.append(point)

        self.plan = plan

    def _srv_init(self):
        timeout = rospy.Duration(5)
        # Wait until service exists or timeout
        try:
            self.load_controller_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller list service. Msg: {}".format(err))
            sys.exit(-1)

    def _get_state_validity(self, constraints=None):
        '''
        RobotState robot_state
        string group_name
        Constraints constraints

        ---

        bool valid
        ContactInformation[] contacts
        CostSource[] cost_sources
        ConstraintEvalResult[] constraint_result
        '''

        self.robot_state.joint_state = self.move_group_cmd.get_current_state().joint_state

        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.robot_state
        gsvr.group_name = self.group_name
        if constraints is not None:
            gsvr.constraints = constraints
        # result = self.state_valid_srv.call(gsvr)
        result = None
        return result

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

    def _moveit_execute(self):
        self.move_group_cmd.execute(self.plan, wait=False)
        while not self.done and not self._update_and_check():
            if self.change_flag:
                self.change_flag = False
                self.move_group_cmd.execute(self.plan, wait=False)
            rospy.sleep(0.05)
        self.clear_targets()

    def _client_execute(self):
        record = []
        joints = Float64MultiArray()
        start_time = rospy.Time.now().to_sec()
        for point in self.plan.joint_trajectory.points:
            if self.mode == 1:
                joints.data = point.positions
            elif self.mode == 2:
                joints.data = point.velocities
            # temp = joints.data
            # joints.data[0] = temp[2]
            # joints.data[2] = temp[0]
            self.controller_client.publish(joints)
            duration = point.time_from_start.to_sec() - (rospy.Time.now().to_sec() - start_time)
            rospy.sleep(duration)
            record.append(duration)
        self.clear_targets()

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


    # def _states_update_cb(self, msg):
    #     actual_joints = list(msg.position)
    #     actual_speed = np.array(msg.velocity)
    #     actual_effort = np.array(msg.effort)
    #     jacobian = self.move_group_cmd.get_jacobian_matrix(actual_joints)
    #
    #     self.eef_speed = np.dot(jacobian, actual_speed)
    #     self.eef_effort = np.dot(jacobian, actual_effort)
    #
    #     # self.eef_x_pub.publish(self.eef_speed[0])
    #     # self.eef_y_pub.publish(self.eef_speed[1])
    #     # self.eef_z_pub.publish(self.eef_speed[2])
    #     markers = MarkerArray()
    #     marker = Marker()
    #
    #     marker.header.frame_id = "tool0"
    #     marker.header.stamp = rospy.Time.now()
    #     marker.ns = "speed"
    #     marker.id = 0
    #     # marker.color = getColor(self.color, alpha=self.alpha)
    #     marker.type = Marker.ARROW
    #     pose = Pose()
    #     pose.orientation.w = 1
    #     marker.pose = pose
    #     point = Point()
    #     point.z = 0
    #     marker.points.append(point)
    #     point = Point()
    #     point.x = self.eef_speed[0]
    #     point.y = self.eef_speed[1]
    #     point.z = self.eef_speed[2] + 0
    #     marker.points.append(point)
    #     marker.scale.x = 0.1
    #     marker.scale.y = 0.15
    #     # marker.scale.x = self.eef_speed[0]
    #     # marker.scale.y = self.eef_speed[1]
    #     # marker.scale.z = self.eef_speed[2]
    #     marker.color.a = 1
    #
    #     markers.markers.append(deepcopy(marker))
    #     marker.id = 1
    #     marker.color.b = 1
    #     marker.points.clear()
    #     point = Point()
    #     point.z = 0
    #     marker.points.append(point)
    #     point = Point()
    #     point.x = self.eef_effort[0]
    #     point.y = self.eef_effort[1]
    #     point.z = self.eef_effort[2]
    #     marker.points.append(point)
    #     markers.markers.append(marker)
    #     self.eef_pub.publish(markers)
