import numpy as np
import rospy
from geometry_msgs.msg import Pose
from move_base import MoveGroup
import tf.transformations as tft
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from helper_func import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from ur3e_kinematic import ur3e_fk
from copy import copy

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
        self.B = np.eye(6)*3.0
        self.K = np.eye(6)*1.0

        # Desired states
        self.x_d = np.zeros((6,))
        self.x_d_dot = np.zeros((6,))

        # Current states
        self.x_n = np.zeros((6,))
        self.x_n_dot = np.zeros((6,))
        self.q_n = np.zeros((6,))
        self.q_n_dot = np.zeros((6,))
        # The external force
        self.F_e = np.zeros((6,))
        # Jacobian
        self.jacobian = np.zeros((6, 6))

        # Work parameter
        self.rate = None
        self.freq = 20

        # Callback
        self.callback = None

        # Subscriber
        rospy.Subscriber("/joint_states", JointState, self._states_update_cb, queue_size=1)
        self.ur_solver = ur3e_fk()

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
        self.x_d = np.copy(self.x_n)
        joints = Float64MultiArray()
        self.rate = rospy.Rate(self.freq)
        while not self.done:
            with self.lock:
                q_n = np.copy(self.q_n)
                x_e_dot = self.x_n_dot - self.x_d_dot
                x_e = poseOperation(self.x_n, self.x_d, mode=2)
            x_e_dot_dot = np.dot(np.linalg.inv(self.M),
                                 (self.F_e - np.dot(self.B, x_e_dot) - np.dot(self.K, x_e)))
            # Inverse
            x_e_dot_next = x_e_dot_dot/self.freq + x_e_dot
            x_e_next = poseOperation(x_e_dot_next/self.freq, x_e, mode=1)
            x_n_next = poseOperation(x_e_next, self.x_d, mode=1)

            q_n_next = self.ur_solver.inverse(x_n_next, q_n, 'rpy')
            if isinstance(q_n_next, type(None)):
                rospy.loginfo("No solution")
                continue
            if any(abs(q_n_next - q_n) > 0.2):
                rospy.loginfo("Error inverse kinematics")
                continue

            if self.mode == 1:
                joints.data = q_n_next
                self.controller_client.publish(joints)
            elif self.mode == 2:
                q_n_dot_next = (q_n_next - q_n)*self.freq
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
                # Position
                _position = msg.position
                self.q_n[:] = [_position[2], _position[1], _position[0], *_position[3:]]
                _x_n = self.ur_solver.forward(self.q_n, "rpy")
                self.x_n[:] = _x_n

                q_n_test = self.ur_solver.inverse(_x_n, self.q_n, 'rpy')
                # if (abs(q_n_test - self.q_n) > 1e-4).any:
                #     rospy.loginfo("Error inverse kinematics")
                # Jacobian
                self.jacobian[:] = self._compute_jacobian()

                # Velocity
                _velocity = msg.velocity
                self.q_n_dot[:] = [_velocity[2], _velocity[1], _velocity[0], *_velocity[3:]]
                self.x_n_dot[:] = np.dot(self.jacobian, self.q_n_dot)

    def set_callback(self, callback):
        self.callback = callback

    def _compute_jacobian(self, delta=0.05):
        t_n = self.ur_solver.forward(self.q_n, 'matrix')
        delta_q = delta/180*np.pi

        jacobian = np.zeros((6, 6))
        for i in range(6):
            _q_n_next = np.copy(self.q_n)
            _q_n_next[i] += delta_q
            t_n_next = self.ur_solver.forward(_q_n_next, 'matrix')

            delta_t = t_n_next[:3, 3] - t_n[:3, 3]
            jacobian[:3, i] = delta_t / delta_q

            delta_o = np.array(tft.euler_from_matrix(np.dot(t_n_next[:3, :3],
                                                            np.transpose(t_n[:3, :3]))))
            jacobian[3:, i] = delta_o / delta_q

        return jacobian


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

