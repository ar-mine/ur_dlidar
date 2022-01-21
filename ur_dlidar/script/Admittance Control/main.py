import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

from admittance_control import AdmittanceControl
from force_generate import ProximityForce


def callback():
    if not force_thread.dlidar_name == "world":
        unit_matrix = np.eye(4)
        unit_matrix[0, 3] = 1
        t, r = force_thread.tf_listener.lookupTransform("base", force_thread.dlidar_name, time=rospy.Time(0))
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R.from_quat(r).as_matrix()
        transform_matrix[:3, 3] = t
        transformed_unit = np.dot(transform_matrix, unit_matrix)
        unit_vector = transformed_unit[:3, 3] - np.array(t)
        ad_control_thread.F_e[:3] = -unit_vector * force_thread.force
    else:
        ad_control_thread.F_e = [0] * 6


if __name__ == "__main__":
    node_name = "AdmittanceControl"
    rospy.init_node(node_name)

    force_thread = ProximityForce(node_name, viz_flag=True)
    ad_control_thread = AdmittanceControl(node_name, mode=1)

    ad_control_thread.set_callback(callback)

    force_thread.start()
    ad_control_thread.start()
