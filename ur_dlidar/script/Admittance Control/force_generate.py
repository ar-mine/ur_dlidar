import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8MultiArray
import numpy as np
from threading import Thread, RLock
from visualization_msgs.msg import Marker, MarkerArray
import tf
from geometry_msgs.msg import Pose, Point


class ProximityForce(Thread):
    def __init__(self, prefix: str, viz_flag: bool):
        super(ProximityForce, self).__init__()
        self.distance = np.zeros((8,))
        self.mask = np.zeros((8,))
        self.lock = RLock()
        self.max_d = 0.55
        self.min_d = 0.05
        self.stop = False
        self.force = 0
        self.rate = None
        self.viz_flag = viz_flag
        self.dlidar_name = "world"

        # Listener
        rospy.Subscriber("dlidar_data", Range, self._distance_cb, queue_size=1)
        rospy.Subscriber("fcl/mask_pub", Int8MultiArray, self._mask_cb, queue_size=1)
        # Publisher
        if viz_flag:
            self.viz_pub = rospy.Publisher(prefix+"/viz_pub", MarkerArray, queue_size=1)
        # TF listener
        self.tf_listener = tf.TransformListener()

        rospy.sleep(2)

    def run(self):
        self.rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                distance = self.distance
                mask = self.mask

            for i, m in enumerate(mask):
                if m:
                    distance[i] = 1.0

            distance[distance == 0] = 1.0
            min_idx = np.argmin(distance)
            min_cls = distance[min_idx]
            if min_cls < self.max_d:
                self.force = 1 - (min_cls-self.min_d) / (self.max_d-self.min_d)
                self.dlidar_name = "dlidar"+str(min_idx)
            else:
                self.dlidar_name = "world"
                self.force = 0
            if self.viz_flag:
                self._force_viz(self.dlidar_name)
            self.rate.sleep()

    def _distance_cb(self, msg):
        with self.lock:
            idx = int(msg.header.frame_id[-1])
            self.distance[idx] = msg.range

    def _mask_cb(self, msg):
        with self.lock:
            self.mask = msg.data

    def _force_viz(self, frame_id):
        markers = MarkerArray()
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "force"
        marker.id = 0
        marker.type = Marker.ARROW

        pose = Pose()
        pose.orientation.w = 1
        marker.pose = pose

        point = Point()
        point.z = 0
        marker.points.append(point)

        point = Point()
        point.x = self.force
        marker.points.append(point)
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.color.a = 1

        markers.markers.append(marker)
        self.viz_pub.publish(markers)


# Test
if __name__ == "__main__":
    node_name = "test"
    rospy.init_node(node_name)

    force_thread = ProximityForce(node_name, viz_flag=True)

    force_thread.start()
