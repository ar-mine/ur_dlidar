import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF
import urdf_parser_py.urdf
import tf
import tf.transformations as tft
import numpy as np
from helper_func import *
from threading import Thread
import fcl
import trimesh
from fcl_visualization import Fcl_viz
import copy


class Cls_viz(Thread):
    def __init__(self):
        super(Cls_viz, self).__init__()
        # Idx record for updating
        self.count = -1
        # URDF description from ros parameter sever
        self.robot = URDF.from_parameter_server()
        # The markers to be published for visualizing collision
        # Save transformation array to decrease computation cost
        self.viz_array = MarkerArray()
        self.cls_array = []
        self.cls_points = []
        self.viz_obj_array = []
        # Basic world link
        self.world = "world"

        # After initialize tf listener, leave few seconds for keeping it stable
        self.tf_listener = tf.TransformListener()
        self.vis_pub = rospy.Publisher("viz_pub", MarkerArray, queue_size=10)
        self.cls_pub = rospy.Publisher("cls_pub", MarkerArray, queue_size=10)
        rospy.sleep(2)

        # Init the pose of collision
        self.robot_init()

        self.done = False

    def robot_init(self):
        viz_obj = Fcl_viz("cone", ns="robot", idx=self.count, color="GREY", scale=[np.pi/12, 1], link_name="world", alpha=0.7)
        viz_obj.origin_matrix = np.dot(tft.translation_matrix([0.56, 0, 0]),
                                       tft.quaternion_matrix([0, 0.707, 0, 0.707]))
        self.viz_obj_array.append(viz_obj)
        self.viz_array.markers.append(viz_obj.marker)
        self.cls_array.append(viz_obj.fobj)

        self.count += 1
        viz_obj = Fcl_viz("box", ns="robot", idx=self.count, color="ORANGE", scale=[1, 1, 0.2], link_name="world", alpha=0.3)
        viz_obj.origin_matrix = np.dot(tft.translation_matrix([0.56, 0, 0]),
                                       tft.quaternion_matrix([0, 0.707, 0, 0.707]))
        self.viz_obj_array.append(viz_obj)
        self.viz_array.markers.append(viz_obj.marker)
        self.cls_array.append(viz_obj.fobj)

    def run(self):
        while not self.done:
            self.update()

            o1 = self.cls_array[0]
            o2 = self.cls_array[1]
            self.collision_com()
            # self.cls_points = []
            # for i in range(1, self.count + 1):
            #     o1 = self.cls_array[0]
            #     o2 = self.cls_array[i]
            #     ret = self.collision_pub(o1, o2, i)
            #     if ret:
            #         self.cls_points.append(ret)
            # self.vis_pub.publish(self.cls_points)
            rospy.sleep(0.005)

    def stop(self):
        self.done = True

    def update(self):

        self.viz_obj_array[0].update(list2Pose([0, 0, 0], [0, 0, 0, 1]))

        self.viz_obj_array[1].update(list2Pose([0, 0, 0], [0, 0, 0, 1]))

        self.vis_pub.publish(self.viz_array)

    def collision_pub(self, o1, o2):
        request = fcl.CollisionRequest()
        request.enable_contact = True
        result = fcl.CollisionResult()

        ret = fcl.collide(o1, o2, request, result)
        if ret:
            marker_array = MarkerArray()
            marker = Marker()

            marker.type = Marker.SPHERE

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.header.frame_id = self.world
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cls_points"
            marker.color = getColor("RED")
            marker.action = Marker.ADD

            for i, contact in enumerate(result.contacts):
                marker_copy = copy.copy(marker)
                # Update marker idx
                marker_copy.id = i
                marker_copy.pose = list2Pose(result.contacts[0].pos, [0, 0, 0, 1.0])
                marker_array.markers.append(marker_copy)
            self.cls_pub.publish(marker_array)
        else:
            return None

    def collision_com(self):
        objs1 = [self.cls_array[0]]
        objs2 = [self.cls_array[1]]

        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()

        manager1.registerObjects(objs1)
        manager2.registerObjects(objs2)

        manager1.setup()
        manager2.setup()

        req = fcl.CollisionRequest(num_max_contacts=1000, enable_contact=True)

        # =====================================================================
        # Managed many to many collision checking
        # =====================================================================
        rdata = fcl.CollisionData(request=req)
        manager1.collide(manager2, rdata, fcl.defaultCollisionCallback)

        self.collision_pub2(rdata)

    def collision_pub2(self, rdata):
        marker_array = MarkerArray()
        marker = Marker()

        marker.type = Marker.SPHERE

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.header.frame_id = self.world
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cls_points"
        marker.color = getColor("RED")
        marker.action = Marker.ADD

        for i, contact in enumerate(rdata.result.contacts):
            marker_copy = copy.copy(marker)
            # Update marker idx
            marker_copy.id = i
            marker_copy.pose = list2Pose(contact.pos, [0, 0, 0, 1.0])
            marker_array.markers.append(marker_copy)

        self.cls_pub.publish(marker_array)


def on_shutdown():
    if not viz_node.done:
        viz_node.stop()


if __name__ == "__main__":
    rospy.init_node("fcl_cls")
    rospy.on_shutdown(on_shutdown)

    viz_node = Cls_viz()
    viz_node.run()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
