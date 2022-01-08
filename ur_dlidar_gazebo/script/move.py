import sys
from math import dist, fabs, cos
from threading import Thread

# ROS
import rospy
from std_msgs.msg import Float32
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

# moveit
import moveit_commander
from moveit_commander.conversions import pose_to_list, list_to_pose
from helper_func import *
from moveit_msgs.msg import CollisionObject, RobotTrajectory
from moveit_msgs.srv import GetStateValidityRequest

# custom
from config import Pre_pose


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

        # Set topic listener
        rospy.Subscriber("move_group/speed_scaling", Float32, self._speed_scaling, queue_size=1)

        # Updated parameters
        self.speed_scaling = 1
        self.speed_flag = False
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
                    if self.speed_flag:
                        self._scale_cartesian_path(self.pose_goals)
                        self.plan.joint_trajectory.points[0].positions = self.move_group_cmd.get_current_joint_values()
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

    def _all_close(self, tolerance=0.05):
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
        self.execute_flag = False
        rospy.loginfo("Set target successful")

    def set_cartesian_targets(self, goal: Pose):
        rospy.loginfo("Begin set target")
        self._scale_cartesian_path(goal)
        self.pose_goals = goal
        self.goal_type = 2
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
        self.execute_flag = True
        rospy.loginfo("Clear targets successful")

    def wait_cb(self, callback=None):
        rospy.loginfo("Waiting")
        while not self.execute_flag and not self.done:
            if callback is not None:
                callback()
        rospy.sleep(0.5)
        rospy.loginfo("Waiting finished")

    def plan_cartesian_path(self, goal, wait=False):
        move_group = self.move_group_cmd
        waypoints = goal

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        ret = self.move_group_cmd.execute(plan, wait)
        print(ret)

    def go_to_goal(self, goal, wait=False):
        if type(goal) is JointState:
            self.goal_type = 1
        elif type(goal) is Pose:
            self.goal_type = 2
        ret, plan, planning_time, error_code = self.move_group_cmd.plan(goal)
        self.move_group_cmd.go(wait=wait)

    def check_close(self):
        if self.goal_type == 1:
            joint_current = self.get_joint_state(False)
            return all_close(self.target, joint_current)
        elif self.goal_type == 2:
            pose_current = self.get_pose()
            return all_close(self.target, pose_current)

    def pre_set_pose(self, name):
        self.go_to_goal(Pre_pose[name], wait=True)

    def home(self):
        self.go_to_goal(Pre_pose["stand2"], wait=True)

    def place(self):
        self.go_to_goal(Pre_pose["place"], wait=True)

    def plan_execute(self, wait=False):
        self.move_group_cmd.execute(self.plan, wait)

    def _speed_scaling(self, msg):
        speed_scaling = msg.data
        if not self.speed_scaling == speed_scaling:
            rospy.loginfo("Change speed")
            self.speed_scaling = speed_scaling
            self.speed_flag = True
            rospy.sleep(0.05)
        else:
            self.speed_flag = False

    # todo
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # # Publish
        # display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group_cmd

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.object_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_collision(self, obj_name, positions, timeout=4):
        scene = self.scene
        self.object_name = obj_name

        # Create object
        obj = SolidPrimitive()
        obj.type = obj.SPHERE
        obj.dimensions = [0.1]

        obj2 = SolidPrimitive()
        obj2.type = obj.SPHERE
        obj2.dimensions = [0.15]

        # Add object as collision
        obj_msg = CollisionObject()

        for idx, pos in enumerate(positions):
            # Create object position
            obj_pose = Pose()
            obj_pose.orientation.w = 1.0
            obj_pose.position.x = pos.z / 1000.0
            obj_pose.position.y = pos.x / -1000.0
            obj_pose.position.z = pos.y / 1000

            if idx in [7, 8, 12, 13]:
                obj_msg.primitives.append(obj2)
            else:
                obj_msg.primitives.append(obj)
            obj_msg.primitive_poses.append(obj_pose)

        obj_msg.header.frame_id = "nuitrack_link"
        obj_msg.id = obj_name
        obj_msg.operation = CollisionObject.ADD

        # Add collision info to planning scene
        scene.add_object(obj_msg)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    # private:
    def _skl_callback(self, msg):
        if msg.skeletons:
            skl_data = msg.skeletons[0]
            self.add_collision("hand", skl_data.joint_pos)

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
        result = self.state_valid_srv.call(gsvr)
        return result
