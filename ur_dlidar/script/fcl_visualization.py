import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF
import urdf_parser_py.urdf
import tf.transformations as tft
import numpy as np
from helper_func import *
import fcl
import trimesh


class Fcl_viz:
    def __init__(self, fin, ns: str, idx, color: str, world: str = "world", **kwargs):
        """
            mode=0: shape mode
            mode=1: link to shape mode
        """
        if isinstance(fin, str):
            self.mode = 0
        elif isinstance(fin, urdf_parser_py.urdf.Link):
            self.mode = 1
        else:
            raise "Invalid input!"

        # Adjust to generalize link and shape mode
        if self.mode:
            link = fin
            self.link_name = link.name
            if link.collision.origin:
                xyz = link.collision.origin.xyz
                rpy = link.collision.origin.rpy
                self.origin_matrix = np.dot(tft.translation_matrix(xyz), tft.euler_matrix(rpy[0], rpy[1], rpy[2]))
            else:
                self.origin_matrix = None
            if isinstance(link.collision.geometry, urdf_parser_py.urdf.Mesh):
                self.type = "mesh"
                self.mesh_resource = link.collision.geometry.filename
                self.scale = [1, 1, 1]
            elif isinstance(link.collision.geometry, urdf_parser_py.urdf.Box):
                self.type = "box"
                self.scale = link.collision.geometry.size
            elif isinstance(link.collision.geometry, urdf_parser_py.urdf.Cylinder):
                self.type = "cylinder"
                self.scale = [link.collision.geometry.radius, link.collision.geometry.length]
            else:
                raise "Not support shape when convert link"
        else:
            self.origin_matrix = None
            self.type = fin
            self.scale = kwargs["scale"]
            self.link_name = kwargs["link_name"]
            if self.type == "mesh":
                self.mesh_resource = kwargs["mesh_path"]

        # Get the necessary info
        self.ns = ns
        self.world = world
        self.id = idx
        self.color = color
        self.pose = Pose()
        self.tf_matrix = np.eye(4)
        if "alpha" in kwargs.keys():
            self.alpha = kwargs["alpha"]
        else:
            self.alpha = 1.0
        self.marker = self._marker_obj()
        self.fobj, self.geom = self._cls_obj()

    def _marker_obj(self):
        marker = Marker()

        marker.header.frame_id = self.world
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.ns
        marker.id = self.id
        marker.color = getColor(self.color, alpha=self.alpha)
        marker.pose = self.pose

        if self.type == "mesh":
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = self.mesh_resource

            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
        elif self.type == "box":
            marker.type = Marker.CUBE

            marker.scale.x = self.scale[0]
            marker.scale.y = self.scale[1]
            marker.scale.z = self.scale[2]
        elif self.type == "cylinder":
            marker.type = Marker.CYLINDER

            marker.scale.x = self.scale[0]
            marker.scale.y = self.scale[0]
            marker.scale.z = self.scale[1]
        elif self.type == "cone":
            marker = ConeMarker(marker, self.scale[0], self.scale[1])
        else:
            raise "Invalid type when generate marker."

        return marker

    def _cls_obj(self):
        if self.type == "mesh":
            if self.mode:
                stl_path = "urdf" + self.mesh_resource[24:]
                mesh = trimesh.load(stl_path)
            else:
                mesh = trimesh.load(self.mesh_resource)

            model = fcl.BVHModel()
            model.beginModel(len(mesh.vertices), len(mesh.faces))
            model.addSubModel(mesh.vertices, mesh.faces)
            model.endModel()
        elif self.type == "box":
            """ x, y, z"""
            model = fcl.Box(*self.scale)
        elif self.type == "cylinder":
            """ radius, lz"""
            model = fcl.Cylinder(*self.scale)
        elif self.type == "cone":
            vertices, faces = ConeList(self.scale[0], self.scale[1])
            model = fcl.BVHModel()
            model.beginModel(len(vertices), len(faces))
            model.addSubModel(vertices, faces)
            model.endModel()
        else:
            raise "Invalid type when generate fcl object."

        position, orientation = Pose2list(self.pose)
        ftf = fcl.Transform(orientation, position)
        obj = fcl.CollisionObject(model, ftf)
        return obj, model

    def _pose_adjust(self):
        position, orientation = Pose2list(self.pose)
        transf_matrix = np.dot(tft.translation_matrix(position), tft.quaternion_matrix(orientation))
        if isinstance(self.origin_matrix, np.ndarray):
            q_matrix = np.dot(transf_matrix, self.origin_matrix)
            position = tft.translation_from_matrix(q_matrix)
            orientation = normalizeQuaternion(tft.quaternion_from_matrix(q_matrix))
            self.pose = list2Pose(position, orientation)
            self.tf_matrix = q_matrix
        else:
            self.tf_matrix = transf_matrix

    def update(self, pose: Pose):
        self.pose = pose

        self._pose_adjust()

        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose = self.pose

        ftf = fcl.Transform(self.tf_matrix[:3, :3], self.tf_matrix[:3, 3])
        self.fobj.setTransform(ftf)


def ConeMarker(marker, angle: float, scale: float):
    triangle_marker = marker

    triangle_marker.type = Marker.TRIANGLE_LIST

    vertices, faces = ConeList(angle, scale)

    for face in faces:
        p = [Point(), Point(), Point()]

        p[0].x = vertices[face[0], 0]
        p[0].y = vertices[face[0], 1]
        p[0].z = vertices[face[0], 2]

        p[1].x = vertices[face[1], 0]
        p[1].y = vertices[face[1], 1]
        p[1].z = vertices[face[1], 2]

        p[2].x = vertices[face[2], 0]
        p[2].y = vertices[face[2], 1]
        p[2].z = vertices[face[2], 2]

        triangle_marker.points.append(p[0])
        triangle_marker.points.append(p[1])
        triangle_marker.points.append(p[2])

    triangle_marker.scale.x = 1.0
    triangle_marker.scale.y = 1.0
    triangle_marker.scale.z = 1.0

    return triangle_marker


def ConeList(angle: float, scale: float, seg: int = 16):
    seg2 = 2*seg
    delta_theta = np.pi / seg
    theta = 0
    vertices = np.zeros((2+seg2, 3))

    vertices[0] = [0, 0, scale/2]
    vertices[seg2+1] = [0, 0, -scale/2]
    faces = []

    for i in range(1, seg2+1):
        vertices[i] = [scale * np.cos(theta) * np.tan(angle), scale * np.sin(theta) * np.tan(angle), scale/2]
        theta += delta_theta
        if i == seg2:
            faces.append([0, i, 1])
            faces.append([seg2 + 1, i, 1])
        else:
            faces.append([0, i, i + 1])
            faces.append([seg2 + 1, i, i + 1])

    return vertices, faces
