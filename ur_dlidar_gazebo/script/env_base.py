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
        self.count = 0
        # URDF description from ros parameter sever
        self.robot = URDF.from_parameter_server()
        # The markers to be published for visualizing collision
        # Save transformation array to decrease computation cost
        self.viz_array = MarkerArray()
        self.cls_array = []
        self.viz_obj_array = []
        self.link_name_array = []
        self.geometry_array = []
        # Basic world link
        self.world = "world"

        # After initialize tf listener, leave few seconds for keeping it stable
        self.tf_listener = tf.TransformListener()
        self.vis_pub = rospy.Publisher("viz_pub", MarkerArray, queue_size=10)
        self.cls_pub = rospy.Publisher("cls_pub", MarkerArray, queue_size=10)
        rospy.sleep(2)

        # Init the pose of collision
        self.robot_init()

        self.manager1 = fcl.DynamicAABBTreeCollisionManager()
        self.manager2 = fcl.DynamicAABBTreeCollisionManager()
        self.geom_id_to_name1 = {}
        self.geom_id_to_name2 = {}
        self.cls_init()

        self.done = False

    def cls_init(self):
        geoms1 = self.geometry_array[17:self.count]
        geoms2 = self.geometry_array[:9]

        objs1 = self.cls_array[17:self.count]
        objs2 = self.cls_array[:9]

        names1 = self.link_name_array[17:self.count]
        names2 = self.link_name_array[:9]

        # Create map from geometry IDs to string names
        self.geom_id_to_name1 = {id(geom): name for geom, name in zip(geoms1, names1)}
        self.geom_id_to_name2 = {id(geom): name for geom, name in zip(geoms2, names2)}

        self.manager1.registerObjects(objs1)
        self.manager2.registerObjects(objs2)

    def robot_init(self):
        # Get all the links from URDF
        links = self.robot.links
        for link in links:
            if not link.collision:
                continue
            viz_obj = Fcl_viz(link, ns="robot", idx=self.count, color="GREY", alpha=1.0)

            self._list_update(viz_obj)

        dlidar_num = 8
        for i in range(dlidar_num):
            link_name = "dlidar{}".format(i)

            viz_obj = Fcl_viz("cone", ns="dlidar", idx=self.count,
                              color="YELLOW", scale=[np.pi/10, 1.0], link_name=link_name, alpha=0.3)
            viz_obj.origin_matrix = np.dot(tft.translation_matrix([0.52, 0, 0]),
                                           tft.quaternion_matrix([0, 0.707, 0, 0.707]))
            self._list_update(viz_obj)

    def _list_update(self, viz_obj):
        self.viz_obj_array.append(viz_obj)
        self.viz_array.markers.append(viz_obj.marker)
        self.cls_array.append(viz_obj.fobj)
        self.link_name_array.append(viz_obj.link_name)
        self.geometry_array.append(viz_obj.geom)
        self.count += 1

    def run(self):
        marker_array = MarkerArray()

        while not self.done:
            self.update()

            marker_array.markers.clear()
            for i in range(17, self.count):
                o1 = self.cls_array[i]
                for j in range(9):
                    o2 = self.cls_array[j]
                    ret = self.distance_pub(o1, o2, i * 10 + j)
                    if ret:
                        marker_array.markers.append(ret)
                        rospy.loginfo("{} collides with {}".format(self.viz_obj_array[i].link_name,
                                                                   self.viz_obj_array[j].link_name))

            self.cls_pub.publish(marker_array)

            # marker_array.markers.clear()
            # for i in range(17, self.count):
            #     o1 = self.cls_array[i]
            #     for j in range(9):
            #         o2 = self.cls_array[j]
            #         ret = self.collision_pub(o1, o2, i*10+j)
            #         if ret:
            #             marker_array.markers.append(ret)
            #             rospy.loginfo("{} collides with {}".format(self.viz_obj_array[i].link_name,
            #                                                        self.viz_obj_array[j].link_name))
            # self.cls_pub.publish(marker_array)

            # self.collision_com()

            rospy.sleep(0.1)

    def stop(self):
        self.done = True

    def update(self):
        for i, vobj in enumerate(self.viz_obj_array):
            # Get pose from tf transformation
            position, orientation = self.tf_listener.lookupTransform("world", vobj.link_name, time=rospy.Time(0))
            self.viz_obj_array[i].update(list2Pose(position, orientation))
            self.cls_array[i] = vobj.fobj
        self.vis_pub.publish(self.viz_array)

    def distance_pub(self, o1, o2, i):
        request = fcl.DistanceRequest()
        request.enable_nearest_points = True
        result = fcl.DistanceResult()

        ret = fcl.distance(o1, o2, request, result)
        if ret < 0.01:
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

            marker_copy = copy.copy(marker)
            # Update marker idx
            marker_copy.id = i
            marker_copy.pose = list2Pose(result.nearest_points[0], [0, 0, 0, 1.0])
            return marker_copy
        else:
            return None

    def collision_pub(self, o1, o2, i):
        request = fcl.CollisionRequest()
        request.enable_contact = True
        result = fcl.CollisionResult()

        ret = fcl.collide(o1, o2, request, result)
        if ret:
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

            marker_copy = copy.copy(marker)
            # Update marker idx
            marker_copy.id = i
            marker_copy.pose = list2Pose(result.contacts[0].pos, [0, 0, 0, 1.0])
            return marker_copy
        else:
            return None

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

    def collision_com(self):
        self.manager1.setup()
        self.manager2.setup()

        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request=req)
        self.manager1.collide(self.manager2, rdata, fcl.defaultCollisionCallback)

        # Extract collision data from contacts and use that to infer set of
        # objects that are in collision
        objs_in_collision = set()

        for contact in rdata.result.contacts:
            # Extract collision geometries that are in contact
            coll_geom_1 = contact.o1
            coll_geom_2 = contact.o2

            # Get their names
            coll_names = [self.geom_id_to_name1[id(coll_geom_1)],
                          self.geom_id_to_name2[id(coll_geom_2)]]
            coll_names = tuple(sorted(coll_names))
            objs_in_collision.add(coll_names)

        for coll_pair in objs_in_collision:
            rospy.loginfo('Object {} in collision with object {}!'.format(coll_pair[0], coll_pair[1]))

        self.collision_pub2(rdata)


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
