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
        self.trans_array = []
        self.link_array = []
        self.cls_array = []
        self.cls_points = []
        # Basic world link
        self.world = "world"

        # After initialize tf listener, leave few seconds for keeping it stable
        self.tf_listener = tf.TransformListener()
        self.vis_pub = rospy.Publisher("viz_pub", MarkerArray, queue_size=10)
        rospy.sleep(2)

        # Init the pose of collision
        self.robot_init()
        self.vis_pub.publish(self.viz_array)

        self.done = False

    def robot_init(self):
        # Get all the links from URDF
        links = self.robot.links
        for link in links:
            self.count += 1
            viz_obj = Fcl_viz(link, ns="robot", idx=self.count, color="GREY")
            ret = self.link2marker(link)
            if ret:
                self.viz_array.markers.append(ret)

        dlidar_num = 8
        for i in range(dlidar_num):
            link_name = "dlidar{}".format(i)
            self.link_array.append(link_name)
            self.trans_array.append(None)
            position, orientation = self.tf_listener.lookupTransform("world", link_name, time=rospy.Time(0))

            pose = list2Pose(position, orientation)

            ret = self.ConeMarker(pose, np.pi / 24 * 23, "YELLOW", 1.0)
            self.viz_array.markers.append(ret)

            ret = self.collision_add("cone", [0.268, 1])
            if ret:
                self.cls_array.append(ret)
                fct = fcl.Transform(orientation, position)
                self.cls_array[self.count].setTransform(fct)

    def link2marker(self, link: urdf_parser_py.urdf.Link, action=Marker.ADD):
        """ Generate marker according to the description of link """
        # If there is no collision, it will not be added
        if not link.collision:
            return None

        marker = Marker()

        # Choose marker shape according to URDF
        # If the shape is not in the list, not add it
        if isinstance(link.collision.geometry, urdf_parser_py.urdf.Mesh):
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = link.collision.geometry.filename

            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1

            stl_path = "urdf" + marker.mesh_resource[24:]
            ret = self.collision_add("mesh", stl_path)
            if ret:
                self.cls_array.append(ret)
        elif isinstance(link.collision.geometry, urdf_parser_py.urdf.Box):
            marker.type = Marker.CUBE

            marker.scale.x = link.collision.geometry.size[0]
            marker.scale.y = link.collision.geometry.size[1]
            marker.scale.z = link.collision.geometry.size[2]

            ret = self.collision_add("box", [marker.scale.x, marker.scale.y, marker.scale.z])
            if ret:
                self.cls_array.append(ret)
        elif isinstance(link.collision.geometry, urdf_parser_py.urdf.Cylinder):
            marker.type = Marker.CYLINDER

            marker.scale.x = link.collision.geometry.radius
            marker.scale.y = link.collision.geometry.radius
            marker.scale.z = link.collision.geometry.length

            ret = self.collision_add("cylinder", [marker.scale.x, marker.scale.z])
            if ret:
                self.cls_array.append(ret)
        else:
            return None

        # Basic header info
        marker.header.frame_id = self.world
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"

        # Update marker idx
        marker.id = self.count
        self.count += 1

        marker.action = action

        marker.color = getColor("GREY")

        # Get pose from tf transformation
        position, orientation = self.tf_listener.lookupTransform("world", link.name, time=rospy.Time(0))
        self.trans_array.append(None)
        self.link_array.append(link.name)
        # If there is no bias, it's no need to transform
        if link.collision.origin:
            xyz = link.collision.origin.xyz
            rpy = link.collision.origin.rpy
            origin_matrix = np.dot(tft.translation_matrix(xyz), tft.euler_matrix(rpy[0], rpy[1], rpy[2]))
            transf_matrix = np.dot(tft.translation_matrix(position), tft.quaternion_matrix(orientation))
            q_matrix = np.dot(transf_matrix, origin_matrix)
            self.trans_array[self.count] = origin_matrix
            position = tft.translation_from_matrix(q_matrix)
            orientation = normalizeQuaternion(tft.quaternion_from_matrix(q_matrix))
        marker.pose = list2Pose(position, orientation)

        fct = fcl.Transform(tft.quaternion_matrix(orientation)[:3, :3], position)
        self.cls_array[self.count].setTransform(fct)

        return marker

    def ConeMarker(self, pose: Pose, angle: float, color: str, scale: float):
        triangle_marker = Marker()

        triangle_marker.header.frame_id = "world"
        triangle_marker.header.stamp = rospy.Time.now()

        # Update marker idx
        triangle_marker.id = self.count
        self.count += 1
        
        triangle_marker.type = Marker.TRIANGLE_LIST
        triangle_marker.ns = "dlidar"

        triangle_marker.color = getColor(color, alpha=0.5)

        triangle_marker.pose = pose
        triangle_marker.action = Marker.ADD

        delta_theta = np.pi / 16.0
        theta = 0

        for i in range(32):
            p = [Point(), Point(), Point(), Point()]
            p[0].x = 0
            p[0].y = 0
            p[0].z = 0

            p[1].x = scale
            p[1].y = scale * np.cos(theta) / angle
            p[1].z = scale * np.sin(theta) / angle

            p[2].x = scale
            p[2].y = scale * np.cos(theta + delta_theta) / angle
            p[2].z = scale * np.sin(theta + delta_theta) / angle

            p[3].x = scale
            p[3].y = 0
            p[3].z = 0

            triangle_marker.points.append(p[0])
            triangle_marker.points.append(p[1])
            triangle_marker.points.append(p[2])

            triangle_marker.points.append(p[3])
            triangle_marker.points.append(p[1])
            triangle_marker.points.append(p[2])

            theta += delta_theta

        triangle_marker.scale.x = 1.0
        triangle_marker.scale.y = 1.0
        triangle_marker.scale.z = 1.0

        return triangle_marker

    def run(self):
        while not self.done:
            # self.update()
            self.cls_points = []
            for i in range(1, self.count + 1):
                o1 = self.cls_array[0]
                o2 = self.cls_array[i]
                ret = self.collision_pub(o1, o2, i)
                if ret:
                    self.cls_points.append(ret)
            self.vis_pub.publish(self.cls_points)

            rospy.sleep(0.005)

    def stop(self):
        self.done = True
        # self.join()

    def update(self):
        for i, marker in enumerate(self.viz_array.markers):
            # Basic header info
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.MODIFY

            # Get pose from tf transformation
            position, orientation = self.tf_listener.lookupTransform("world", self.link_array[i], time=rospy.Time(0))
            # If there is no bias, it's no need to transform
            if isinstance(self.trans_array[i], np.ndarray):
                origin_matrix = self.trans_array[i]
                transf_matrix = np.dot(tft.translation_matrix(position), tft.quaternion_matrix(orientation))
                q_matrix = np.dot(transf_matrix, origin_matrix)
                position = tft.translation_from_matrix(q_matrix)
                orientation = normalizeQuaternion(tft.quaternion_from_matrix(q_matrix))
            marker.pose = list2Pose(position, orientation)

        self.vis_pub.publish(self.viz_array)

    def collision_add(self, shape: str, opt=None):
        if shape == "mesh":
            mesh = trimesh.load(opt)

            model = fcl.BVHModel()
            model.beginModel(len(mesh.vertices), len(mesh.faces))
            model.addSubModel(mesh.vertices, mesh.faces)
            model.endModel()
        elif shape == "box":
            """ x, y, z"""
            model = fcl.Box(*opt)
        elif shape == "cylinder":
            """ radius, lz"""
            model = fcl.Cylinder(*opt)
        elif shape == "cone":
            """ radius, lz"""
            model = fcl.Cone(*opt)
        else:
            return None

        ftf = fcl.Transform()
        obj = fcl.CollisionObject(model, ftf)
        return obj

    def collision_pub(self, o1, o2, i):
        request = fcl.CollisionRequest()
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

            # Update marker idx
            marker.id = i

            marker.action = Marker.ADD

            marker.color = getColor("RED")
            marker.pose = list2Pose(result.contacts[0].pos, [0, 0, 0, 1.0])
            return marker
        else:
            return None


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
