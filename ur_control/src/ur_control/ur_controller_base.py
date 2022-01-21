import sys
from threading import Thread, RLock
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ur_msgs.srv import SetSpeedSliderFraction
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Custom
from ur_control import ur3e_kinematics


SUPPORT_LIST = ('ur3e',)


class UR_ControlBase(Thread):
    def __init__(self, prefix: str, version: str, freq: float):
        super(UR_ControlBase, self).__init__()
        """
            A basic class provides real-time joint position, velocity and jacobin matrix.
        """
        # Class flag
        # Thread flag, false for running, true for finished
        self.done = True

        # Provide robot version for custom configurations initialization
        if version not in SUPPORT_LIST:
            rospy.logerr("Not support robot version.")
            sys.exit(-1)
        else:
            self.version_idx = SUPPORT_LIST.index(version)

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
        # ROS service for change speed of ur robot
        self.speed_scaling_srv = rospy.ServiceProxy(
            "/ur_hardware_interface/set_speed_slider", SetSpeedSliderFraction
        )
        # Wait for all services are found
        self._srv_init()

        # Controller client
        # When using group controllers, it is a ros publisher
        # When using trajectory controllers, it is a ros action
        self.controller_client = None

        # Subscriber
        rospy.Subscriber("/joint_states", JointState, self._states_update_cb, queue_size=1)
        self.ur_solver = ur3e_kinematics()

        # Lock for thread
        self.lock = RLock()

        # Class constant
        self.joint_names = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        self.controller_switch_names = (
            "scaled_pos_joint_traj_controller",
            "pos_joint_traj_controller",
            "joint_group_pos_controller",
            "joint_group_vel_controller"
        )

        # Class variable parameters
        # Current states
        self.x_n = np.zeros((6,), dtype=np.float_)
        self.x_n_dot = np.zeros((6,), dtype=np.float_)
        self.q_n = np.zeros((6,), dtype=np.float_)
        self.q_n_dot = np.zeros((6,), dtype=np.float_)
        # Jacobin matrix
        self.jacobin = np.zeros((6, 6), dtype=np.float_)

        # 0 for no group, 1 for pos group, 2 for vel group
        self.controller_mode = 0
        self._controller_state_init()

        # Delay 2s to keep the initialization above stable
        rospy.sleep(2)
        self.rate = rospy.Rate(freq)

    def run(self):
        self.done = False
        while not self.done:
            pass

    def stop(self):
        self.done = True
        if self.controller_mode == 2:
            joints = Float64MultiArray()
            joints.data = [0, 0, 0, 0, 0, 0]
            self.controller_client.publish(joints)

    def switch_controller(self, controller: str):
        ret = True

        if controller not in self.controller_names:
            rospy.logerr("The controller {} doesn't exist.".format(controller))
            ret = False
        elif controller not in self.controller_switch_names:
            rospy.logerr("The controller {} is not switchable.".format(controller))
            ret = False
        else:
            c_idx = self.controller_names.index(controller)
            if self.controller_states[c_idx] == "running":
                rospy.loginfo("The controller {} is running, no need to switch.".format(controller))
            else:
                # If the controller is not loaded, load it first
                if not self.controller_states[c_idx] == "initialized":
                    srv = LoadControllerRequest()
                    srv.name = controller
                    self.load_controller_srv(srv)

                # Stop the controller that can conflict
                stop_list = list(self.controller_switch_names)
                stop_list.remove(controller)
                srv = SwitchControllerRequest()
                srv.stop_controllers = stop_list
                srv.start_controllers = [controller]
                srv.strictness = SwitchControllerRequest.BEST_EFFORT
                self.switch_controller_srv.call(srv)
                rospy.loginfo("The controller {} is switched successfully.".format(controller))

            self.controller_mode = self.controller_switch_names.index(controller)
            if self.controller_mode == 0 or self.controller_mode == 1:
                # 0: scaled_pos_joint_traj_controller
                # 1: pos_joint_traj_controller
                self.controller_client = actionlib.SimpleActionClient(
                    "{}/follow_joint_trajectory".format(controller),
                    FollowJointTrajectoryAction)
            elif self.controller_mode == 2 or self.controller_mode == 3:
                # 2: joint_group_pos_controller
                # 3: joint_group_vel_controller
                self.controller_client = rospy.Publisher("{}/command".format(controller),
                                                         Float64MultiArray,
                                                         queue_size=1)
        return ret

    def move_command(self, goal_point, time=None):
        if self.controller_mode == 0 or self.controller_mode == 1:
            # 0: scaled_pos_joint_traj_controller
            # 1: pos_joint_traj_controller
            point = JointTrajectoryPoint()
            point.positions = goal_point
            point.time_from_start = time

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = self.joint_names
            goal.trajectory.points.append(point)

            self.controller_client.send_goal(goal)
        elif self.controller_mode == 2 or self.controller_mode == 3:
            # 2: joint_group_pos_controller
            # 3: joint_group_vel_controller
            self.controller_client.publish(goal_point)

    # private:
    def _compute_jacobin(self, delta=0.05):
        """
            Compute the jacobin matrix by adding a tiny translation
        """
        _t_n = self.ur_solver.forward(self.q_n, 'matrix')
        delta_q = delta/180*np.pi

        _jacobin = np.zeros((6, 6))
        for i in range(6):
            _q_n_next = np.copy(self.q_n)
            _q_n_next[i] += delta_q
            _t_n_next = self.ur_solver.forward(_q_n_next, 'matrix')

            delta_t = _t_n_next[:3, 3] - _t_n[:3, 3]
            _jacobin[:3, i] = delta_t / delta_q
            delta_o = R.from_matrix(np.dot(_t_n_next[:3, :3], np.transpose(_t_n[:3, :3]))).as_euler("xyz")
            _jacobin[3:, i] = delta_o / delta_q

        return _jacobin

    # Init function
    def _srv_init(self):
        """
            Wait until service exists or timeout.
        """
        timeout = rospy.Duration(5)
        try:
            self.switch_controller_srv.wait_for_service(timeout.to_sec())
            self.load_controller_srv.wait_for_service(timeout.to_sec())
            self.list_controller_srv.wait_for_service(timeout.to_sec())
            # self.speed_scaling_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller list service. Msg: {}".format(err))
            sys.exit(-1)

    def _controller_state_init(self):
        self.controller_list = self.list_controller_srv.call().controller
        self.controller_names = [c.name for c in self.controller_list]
        self.controller_states = [c.state for c in self.controller_list]
        for i, c in enumerate(self.controller_switch_names):
            if c in self.controller_names and self.controller_states[i] == "running":
                self.controller_mode = i
                return
        rospy.logerr("Current running controller cannot be detected.")

    # ROS callback
    def _states_update_cb(self, msg):
        if not self.done:
            with self.lock:
                # Position
                _position = msg.position
                self.q_n[:] = [_position[2], _position[1], _position[0], *_position[3:]]
                _x_n = self.ur_solver.forward(self.q_n, "rpy")
                self.x_n[:] = _x_n

                # Jacobin matrix
                self.jacobin[:] = self._compute_jacobin()

                # Velocity
                _velocity = msg.velocity
                self.q_n_dot[:] = [_velocity[2], _velocity[1], _velocity[0], *_velocity[3:]]
                self.x_n_dot[:] = np.dot(self.jacobin, self.q_n_dot)
                self.rate.sleep()


# Test
if __name__ == "__main__":
    node_name = "test"
    rospy.init_node(node_name)

    force_thread = UR_ControlBase(node_name, 'ur3e', freq=50)

    force_thread.start()
