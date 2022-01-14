import rospy
from move_base import MoveGroup
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
    def __init__(self):
        self.distance = np.zeros([8, ])
        self.mask = []
        self.lock = threading.RLock()
        self.max_d = 0.5
        self.stop = False

        rospy.init_node("move_test")
        rospy.on_shutdown(self._on_shutdown)
        # Listener
        rospy.Subscriber("dlidar_data", Range, self._distance_cb, queue_size=1)
        rospy.Subscriber("fcl/mask_pub", Int8MultiArray, self._mask_cb, queue_size=1)

        self.move_group = MoveGroup()
        self.move_group.start()

    def run(self):
        for i in range(3):
            self.move_group.set_cartesian_targets(goal=list2Pose(pose1[0], pose1[1]))
            self.move_group.wait_cb(self._speed_cb)
            self.move_group.set_cartesian_targets(goal=list2Pose(pose2[0], pose2[1]))
            self.move_group.wait_cb(self._speed_cb)

    def _on_shutdown(self):
        if not self.move_group.done:
            self.move_group.stop()
            self.move_group.join()

    def _distance_cb(self, msg):
        with self.lock:
            idx = int(msg.header.frame_id[-1])
            self.distance[idx] = msg.range

    def _mask_cb(self, msg):
        with self.lock:
            self.mask = msg.data

    def _speed_cb(self):
        with self.lock:
            distance = self.distance
            mask = self.mask

        for m, i in enumerate(mask):
            if m:
                distance[i] = 1.0

        cls_d = min(distance)
        spd_scale = cls_d/self.max_d
        if not self.stop:
            if 0.2 <= spd_scale < 1:
                self.move_group.speed_srv(spd_scale)
            elif spd_scale < 0.2:
                self.move_group.stop_c()
                self.stop = True
            else:
                self.move_group.speed_srv(1.0)
        else:
            if spd_scale >= 1.2:
                self.move_group.speed_srv(1.0)
                self.move_group.resume()
                self.stop = False


if __name__ == "__main__":
    task = Task1()
    task.run()

