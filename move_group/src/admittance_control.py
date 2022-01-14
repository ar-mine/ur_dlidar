import numpy as np
import rospy
from geometry_msgs.msg import Pose
from move_base import MoveGroup
import tf.transformations as tft
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import roslib
roslib.load_manifest("ur_kinematics")
from helper_func import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class AdmittanceControl(MoveGroup):
    def __init__(self, mode):
        super(AdmittanceControl, self).__init__()
        # mode 1 for position, mode 2 for velocity， mode 3 for trajectory
        self.mode = mode
        # Wait
        self.done = True

        # The parameters of AdmittanceControl, which should be tuned
        self.M = np.eye(6)*0.5
        self.B = np.eye(6)*2.0
        self.K = np.eye(6)*1.5

        # Init variables
        # self.x_e_dot_dot = np.zeros((6,))
        # self.x_e_dot = np.zeros((6,))
        # self.x_e = np.zeros((6,))
        self.F_e = np.zeros((6,))

        # Desired states
        self.x_d = self.move_group_cmd.get_current_pose().pose
        self.x_d_position, self.x_d_orientation = pose2pose(self.x_d)
        self.x_d_dot = np.zeros((6,))

        # Current states
        self.x_n = self.move_group_cmd.get_current_pose().pose
        self.x_n_dot = np.zeros((6,))
        self.q_n = np.zeros((6,))
        self.q_n_dot = np.zeros((6,))

        # Work parameter
        self.rate = None
        self.freq = 20

        # Callback
        self.callback = None

        # Subscriber
        rospy.Subscriber("/joint_states", JointState, self._states_update_cb, queue_size=1)
        self.ik_solver = rospy.ServiceProxy(
            "/compute_ik", GetPositionIK
        )
        if mode == 1:
            self.switch_controller("joint_group_pos_controller")
        elif mode == 2:
            self.switch_controller("joint_group_vel_controller")
        elif mode == 3:
            # self.switch_controller("pos_joint_traj_controller")
            self.switch_controller("scaled_pos_joint_traj_controller")

        # Start work
        self.done = False
        # Sleep to make system stable
        rospy.sleep(2)

    def run(self):
        self.rate = rospy.Rate(self.freq)
        joints = Float64MultiArray()
        while not self.done:
            with self.lock:
                x_e_dot = self.x_n_dot - self.x_d_dot
                x_e = poseOperation(self.x_n, self.x_d, mode=2)
            x_e_dot_dot = np.dot(np.linalg.inv(self.M),
                                 (self.F_e - np.dot(self.B, x_e_dot) - np.dot(self.K, x_e)))
            # Inverse
            x_e_dot_next = x_e_dot_dot/self.freq + x_e_dot
            x_e_next = poseOperation(x_e_dot_next/self.freq, x_e, mode=1)
            x_n_next = poseOperation(x_e_next, self.x_d, mode=1)

            q_n_next = self._compute_ik(x_n_next)
            if isinstance(q_n_next, type(None)):
                rospy.loginfo("No solution")
                continue

            if self.mode == 1:
                joints.data = q_n_next
            elif self.mode == 2:
                q_n_dot_next = (q_n_next - self.q_n)*self.freq
                joints.data = q_n_dot_next
                self.controller_client.publish(joints)
            elif self.mode == 3:
                # Create and fill trajectory goal
                if self.rate.remaining().to_sec() > 0:
                    point = JointTrajectoryPoint()
                    point.positions = q_n_next
                    point.time_from_start = self.rate.remaining()

                    goal = FollowJointTrajectoryGoal()
                    goal.trajectory.joint_names = JOINT_NAMES
                    goal.trajectory.points.append(point)

                    self.controller_client.send_goal(goal)
                    # self.controller_client.wait_for_result()

            if self.callback is not None:
                self.callback()
            # rospy.loginfo(self.rate.remaining())
            self.rate.sleep()

    def _states_update_cb(self, msg):
        if not self.done:
            with self.lock:
                robot_state = self.move_group_cmd.get_current_state()

                # Position
                actual_joints = list(robot_state.joint_state.position)
                self.q_n[:] = actual_joints
                self.x_n = self.move_group_cmd.get_current_pose().pose

                # Velocity
                actual_speed = robot_state.joint_state.velocity
                if len(actual_speed) == 0:
                    actual_speed = [0] * 6
                self.q_n_dot[:] = actual_speed
                self.jacobian = self.move_group_cmd.get_jacobian_matrix(actual_joints)
                self.x_n_dot[:] = np.dot(self.jacobian, self.q_n_dot)

    def set_callback(self, callback):
        self.callback = callback

    def _compute_ik(self, pose):
        if isinstance(pose, Pose):
            pose_quat = pose
        else:
            pose_quat = list2Pose(pose[:3], tft.quaternion_from_euler(*pose[3:]))

        req = GetPositionIKRequest()
        req.ik_request.ik_link_name = "wrist_3_link"
        req.ik_request.group_name = self.group_name

        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
        req.ik_request.pose_stamped.pose = pose_quat

        req.ik_request.robot_state = self.robot_cmd.get_current_state()

        res = self.ik_solver.call(req)
        if res.error_code.val != 1:
            rospy.logerr("IK error, code is %d" % res.error_code.val)
            return None

        joints = res.solution.joint_state.position
        return joints


def pose2pose(pose: Pose):
    _position = np.array([pose.position.x, pose.position.y, pose.position.z])
    _orientation = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z, pose.orientation.w])
    return _position, np.array(_orientation)


def poseOperation(pose1, pose2, mode):
    pose = np.zeros((6,))

    if isinstance(pose1, Pose):
        position1, orientation1 = pose2pose(pose1)
    elif isinstance(pose1, np.ndarray):
        position1 = pose1[:3]
        orientation1 = tft.euler_matrix(*pose1[3:])
    else:
        return None
    if isinstance(pose2, Pose):
        position2, orientation2 = pose2pose(pose2)
    elif isinstance(pose2, np.ndarray):
        position2 = pose2[:3]
        orientation2 = tft.euler_matrix(*pose2[3:])
    else:
        return None

    if mode == 1:
        pose[:3] = position1 + position2
        pose[3:] = tft.euler_from_matrix(np.dot(orientation1,
                                                orientation2))

    elif mode == 2:
        pose[:3] = position1 - position2
        pose[3:] = tft.euler_from_matrix(np.dot(orientation1,
                                         np.transpose(orientation2)))
    return pose

