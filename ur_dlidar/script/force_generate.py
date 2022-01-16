import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8MultiArray
import numpy as np
import threading
from visualization_msgs.msg import Marker, MarkerArray
import tf
from geometry_msgs.msg import Pose, Point
from copy import deepcopy
from admittance_control import AdmittanceControl


class ProximityForce:
    def __init__(self, node: str, viz_flag):
        self.distance = np.zeros((8,))
        self.mask = np.zeros((8,))
        self.lock = threading.RLock()
        self.max_d = 0.55
        self.min_d = 0.05
        self.stop = False
        self.force = 0
        self.rate = None
        self.viz_flag = viz_flag

        rospy.init_node(node)
        # Listener
        rospy.Subscriber("dlidar_data", Range, self._distance_cb, queue_size=1)
        rospy.Subscriber("fcl/mask_pub", Int8MultiArray, self._mask_cb, queue_size=1)
        # Publisher
        if viz_flag:
            self.viz_pub = rospy.Publisher(node+"/viz_pub", MarkerArray, queue_size=1)
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
                dlidar_name = "dlidar"+str(min_idx)
                # position, orientation = self.tf_listener.lookupTransform("world", vobj.link_name, time=rospy.Time(0))
            else:
                dlidar_name = "world"
                self.force = 0
            if self.viz_flag:
                self._force_viz(dlidar_name)
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


def callback():
    ad_control.F_e[0] = task.force


if __name__ == "__main__":
    task = ProximityForce("ProximityForce", True)

    ad_control = AdmittanceControl(mode=3)
    ad_control.set_callback(callback)
    # ad_control.start()

    task.run()
