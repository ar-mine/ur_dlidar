from math import cos, sin, pi
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from trac_ik_python.trac_ik import IK


class ur3e_kinematics:
    def __init__(self):
        path = os.path.split(os.path.realpath(__file__))[0]
        parse = ""
        with open(path + "/fk_expression.txt") as f:
            lines = f.readlines()
            if not len(lines) == 12:
                assert "The fk_expression should be 12 lines, but it has {} lines now.".format(len(lines))
            for i, line in enumerate(lines):
                if i == 0:
                    parse += "["
                if i % 4 == 0:
                    parse += "["
                parse += line[:-1]+","
                if i % 4 == 3:
                    parse += "],"
                if i == 11:
                    parse += "]"

        with open(path + "/ur3e.urdf") as f:
            urdf = f.read()
            self.ur_ik = IK("base", "tool0", urdf_string=urdf)
        self.parse = parse

    def forward(self, q, output: str = "matrix"):
        if not len(q) == 6:
            assert "Wrong joint value inputs, the length must be 6, but it is {}".format(len(q))
        q1, q2, q3, q4, q5, q6 = q
        matrix = np.eye(4)
        matrix[:3, :] = eval(self.parse)
        if output == "matrix":
            return matrix
        elif output == "quat":
            return np.array([*matrix[:3, 3], *R.from_matrix(matrix[:3, :3]).as_quat()])
        elif output == "rpy":
            return np.array([*matrix[:3, 3], *R.from_matrix(matrix[:3, :3]).as_euler("xyz")])
        else:
            raise "Not support output type, only matrix, quat and rpy are supported!"

    def inverse(self, pose, guess_state, in_put: str = "quat"):
        if in_put == "quat":
            ret = self.ur_ik.get_ik(list(guess_state), *pose)
            if isinstance(ret, type(None)):
                return ret
            return np.array(ret)
        elif in_put == "rpy":
            ret = self.ur_ik.get_ik(list(guess_state), *pose[:3], *R.from_euler("xyz", pose[3:]).as_quat())
            if isinstance(ret, type(None)):
                return ret
            return np.array(ret)
        else:
            raise "Not support output type, only quat and rpy are supported!"


def fk_test():
    fk_solver = ur3e_kinematics()
    print(fk_solver.forward([0.1630, -1.8588, -1.0072, -1.7438, 1.5692, 3.5175]))
    print(fk_solver.forward([0.1630, -1.8588, -1.0072, -1.7438, 1.5692, 3.5175], "quat"))
    print(fk_solver.forward([0.1630, -1.8588, -1.0072, -1.7438, 1.5692, 3.5175], "rpy"))
    test = fk_solver.forward([0.1630, -1.8588, -1.0072, -1.7438, 1.5692, 3.5175], "rpy")
    print(fk_solver.inverse(test, [0.1630, -1.8588, -1.0072, -1.7438, 1.5692, 3.5175], 'rpy'))


if __name__ == "__main__":
    fk_test()
