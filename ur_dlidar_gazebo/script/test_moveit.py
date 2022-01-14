import rospy
from move_group import MoveGroup
from helper_func import *
from sensor_msgs.msg import Range
from std_msgs.msg import Int8MultiArray
import numpy as np
import threading

"""
    Task1: Follow Cartesian trajectory and repeat for 3 times;
           During the process, it can change speed according to the distance to people or other obstacle;
           And it can even stop when the distance is close enough;
           After distance is larger than a threshold, it will resume;
"""

pose1 = [[0.22052175775906774, 0.40461473160807016, 0.34650637038804216],
         [0.001388437889491262, -0.9995571198473021, 0.019579617502582733, 0.0223668276974118]]

pose2 = [[-0.10036513175876659, -0.19401435065149936, 0.38452191367708144],
         [0.7010768141258071, -0.7124512178169863, -0.001673783714135068, 0.03002934189055441]]


class Task1:
    def __init__(self, rosnode: str = "test"):
        self.distance = np.zeros([8, ])
        self.mask = []
        self.lock = threading.RLock()
        self.max_d = 0.5
        self.stop = False

        rospy.init_node(rosnode)
        rospy.on_shutdown(self._on_shutdown)

        self.move_group = MoveGroup()
        self.move_group.switch_controller("joint_group_pos_controller")
        self.move_group.start()

    def run(self):
        for i in range(1):
            self.move_group.set_cartesian_targets(goal=list2Pose(pose1[0], pose1[1]))
            self.move_group.wait_cb()
            self.move_group.set_cartesian_targets(goal=list2Pose(pose2[0], pose2[1]))
            self.move_group.wait_cb()

    def _on_shutdown(self):
        if not self.move_group.done:
            self.move_group.stop()
            self.move_group.join()


if __name__ == "__main__":
    task = Task1("moveit_test")
    task.run()

