import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from .helper_func import *
from .ur_controller_base import UR_ControlBase


class AdmittanceControl(UR_ControlBase):
    def __init__(self, mode):
        super(AdmittanceControl, self).__init__("ur3e")
        # mode 1 for position, mode 2 for velocity， mode 3 for trajectory
        self.mode = mode

        # The parameters of AdmittanceControl, which should be tuned
        self.M = np.eye(6)*0.5
        self.B = np.eye(6)*3.0
        self.K = np.eye(6)*2.0

        # Desired states
        self.x_d = np.zeros((6,))
        self.x_d_dot = np.zeros((6,))

        # The external force
        self.F_e = np.zeros((6,))

        # Work parameter
        self.rate = None
        self.freq = 30

        # Callback
        self.callback = None

        if mode == 1:
            self.switch_controller("scaled_pos_joint_traj_controller")
        elif mode == 2:
            self.switch_controller("pos_joint_traj_controller")
        elif mode == 3:
            self.switch_controller("joint_group_pos_controller")
        elif mode == 4:
            self.switch_controller("joint_group_vel_controller")

        # Start task
        self.done = False
        # Sleep to make system stable
        rospy.sleep(2)

    def run(self):
        # Assume start point is target point
        self.x_d = np.copy(self.x_n)
        joints = Float64MultiArray()
        self.rate = rospy.Rate(self.freq)

        while not self.done:
            with self.lock:
                q_n = np.copy(self.q_n)
                x_e_dot = self.x_n_dot - self.x_d_dot
                x_e = poseOperation(self.x_n, self.x_d, mode=2)
            # Compute virtual acceleration
            x_e_dot_dot = np.dot(np.linalg.inv(self.M),
                                 (self.F_e - np.dot(self.B, x_e_dot) - np.dot(self.K, x_e)))
            # Integration
            x_e_dot_next = x_e_dot_dot/self.freq + x_e_dot
            x_e_next = poseOperation(x_e_dot_next/self.freq, x_e, mode=1)
            x_n_next = poseOperation(x_e_next, self.x_d, mode=1)

            # Inverse kinematics for joints
            q_n_next = self.ur_solver.inverse(x_n_next, q_n, 'rpy')

            if isinstance(q_n_next, type(None)):
                rospy.loginfo("No solution")
                continue
            if any(abs(q_n_next - q_n) > 0.2):
                rospy.loginfo("Error inverse kinematics")
                continue

            if self.mode == 1 or self.mode == 2:
                # Create and fill trajectory goal
                if self.rate.remaining().to_sec() > 0:
                    self.move_command(q_n_next, self.rate.remaining())
            if self.mode == 3:
                joints.data = q_n_next
                self.move_command(joints)
            elif self.mode == 4:
                q_n_dot_next = (q_n_next - q_n)*self.freq
                joints.data = q_n_dot_next
                self.move_command(joints)

            if self.callback is not None:
                self.callback()

            # print(self.rate.remaining())
            self.rate.sleep()

    def set_callback(self, callback):
        self.callback = callback




