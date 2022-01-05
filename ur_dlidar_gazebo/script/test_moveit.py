import rospy
from move import MoveGroup
from helper_func import *

pose1 = [[0.22052175775906774, 0.40461473160807016, 0.34650637038804216],
         [0.001388437889491262, -0.9995571198473021, 0.019579617502582733, 0.0223668276974118]]

pose2 = [[-0.10036513175876659, -0.19401435065149936, 0.38452191367708144],
         [0.7010768141258071, -0.7124512178169863, -0.001673783714135068, 0.03002934189055441]]

if __name__ == "__main__":
    rospy.init_node("move_test")
    move_group = MoveGroup()
    for i in range(3):
        move_group.plan_cartesian_path(goal=[list2Pose(pose1[0], pose1[1])], wait=True)
        move_group.plan_cartesian_path(goal=[list2Pose(pose2[0], pose2[1])], wait=True)
    # while not rospy.is_shutdown():
    #     print("get pose, false", move_group.get_pose())
    #     print("get pose, true", move_group.get_pose(True))
    #     print("get joint states, false", move_group.get_joint_state())
    #     print("get joint states, true", move_group.get_joint_state(True))
    #     print("\n")
    #     rospy.sleep(0.5)
