import rospy
from move import MoveGroup
from helper_func import *
import time

pose1 = [[0.22052175775906774, 0.40461473160807016, 0.34650637038804216],
         [0.001388437889491262, -0.9995571198473021, 0.019579617502582733, 0.0223668276974118]]

pose2 = [[-0.10036513175876659, -0.19401435065149936, 0.38452191367708144],
         [0.7010768141258071, -0.7124512178169863, -0.001673783714135068, 0.03002934189055441]]


def on_shutdown():
    if not move_group.done:
        move_group.stop()
        move_group.join()


if __name__ == "__main__":
    rospy.init_node("move_test")
    rospy.on_shutdown(on_shutdown)
    move_group = MoveGroup()
    move_group.start()
    for i in range(2):
        move_group.set_cartesian_targets(goal=list2Pose(pose1[0], pose1[1]))
        move_group.wait_cb()
        move_group.set_cartesian_targets(goal=list2Pose(pose2[0], pose2[1]))
        move_group.wait_cb()
