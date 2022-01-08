import numpy as np
import rospy
import actionlib
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from math import dist, fabs, cos
from helper_func import *


class FollowTrajectory(threading.Thread):
    def __init__(self):
        super(FollowTrajectory, self).__init__()

        self.done = False
        self.current_angles = np.zeros((6,))
        self.lock = threading.RLock()

        rospy.Subscriber("/joint_states", JointState, self.joints_cb, queue_size=1)
        self.client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("Action sever ready")
        self.target = self.current_angles

    def joints_cb(self, msg):
        with self.lock:
            current_angles = msg.position
        if len(current_angles) <= 1:
            return
        self.current_angles = np.array([current_angles[2], current_angles[1], current_angles[0], *current_angles[3:]])

    def run(self):
        while not self.done:
            self.traj_client()
            rospy.sleep(0.05)

    def stop(self):
        self.done = True
        self.join()

    def traj_client(self):
        with self.lock:
            current = self.current_angles
            position = self.target

        if all_close(current, position, tolerance=0.02):
            return
        point = JointTrajectoryPoint()
        point.positions = position
        duration = max(abs(current - position)) / 0.5 + 2.0
        point.time_from_start = rospy.Duration(int(duration), (duration - int(duration)) * 50000000)

        traj = JointTrajectory()
        traj.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        traj.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.trajectory.header.stamp = rospy.Time.now()

        self.client.send_goal(goal)
        # self.client.wait_for_result()

        # return self.client.get_result()

    def set_target(self, target):
        if isinstance(target, np.ndarray):
            self.target = target
        else:
            rospy.logerr("The goal should be ndarray type!")


def all_close(goal, actual, tolerance=0.05):
    if type(goal) is list or type(goal) is np.ndarray:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = Pose2list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = Pose2list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

