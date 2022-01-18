from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose


def getColor(color: str, alpha=1.0):
    result = ColorRGBA()

    if color == "RED":
        result.r = 0.8
        result.g = 0.1
        result.b = 0.1
        result.a = alpha
    elif color == "GREEN":
        result.r = 0.1
        result.g = 0.8
        result.b = 0.1
        result.a = alpha
    elif color == "GREY":
        result.r = 0.9
        result.g = 0.9
        result.b = 0.9
        result.a = alpha
    elif color == "DARK_GREY":
        result.r = 0.6
        result.g = 0.6
        result.b = 0.6
        result.a = alpha
    elif color == "WHITE":
        result.r = 1.0
        result.g = 1.0
        result.b = 1.0
        result.a = alpha
    elif color == "ORANGE":
        result.r = 1.0
        result.g = 0.5
        result.b = 0.0
        result.a = alpha
    elif color == "TRANSLUCENT_LIGHT":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.1
    elif color == "TRANSLUCENT":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.25
    elif color == "TRANSLUCENT_DARK":
        result.r = 0.1
        result.g = 0.1
        result.b = 0.1
        result.a = 0.5
    elif color == "BLACK":
        result.r = 0.0
        result.g = 0.0
        result.b = 0.0
        result.a = alpha
    elif color == "YELLOW":
        result.r = 1.0
        result.g = 1.0
        result.b = 0.0
        result.a = alpha
    elif color == "BROWN":
        result.r = 0.597
        result.g = 0.296
        result.b = 0.0
        result.a = alpha
    elif color == "PINK":
        result.r = 1.0
        result.g = 0.4
        result.b = 1
        result.a = alpha
    elif color == "LIME_GREEN":
        result.r = 0.6
        result.g = 1.0
        result.b = 0.2
        result.a = alpha
    elif color == "CLEAR":
        result.r = 1.0
        result.g = 1.0
        result.b = 1.0
        result.a = 0.0
    elif color == "PURPLE":
        result.r = 0.597
        result.g = 0.0
        result.b = 0.597
        result.a = alpha
    elif color == "CYAN":
        result.r = 0.0
        result.g = 1.0
        result.b = 1.0
        result.a = alpha
    elif color == "MAGENTA":
        result.r = 1.0
        result.g = 0.0
        result.b = 1.0
        result.a = alpha
    else:
        result.r = 1.0
        result.g = 1.0
        result.b = 1.0
        result.a = alpha

    return result


def list2Pose(p_list: list, o_list: list):
    p = Pose()
    p.position.x = p_list[0]
    p.position.y = p_list[1]
    p.position.z = p_list[2]
    p.orientation.x = o_list[0]
    p.orientation.y = o_list[1]
    p.orientation.z = o_list[2]
    p.orientation.w = o_list[3]

    return p


def Pose2list(p: Pose):
    p_list = [p.position.x, p.position.y, p.position.z]
    o_list = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    return p_list, o_list


def normalizeQuaternion(quaternion):
    quaternion_msg = quaternion
    norm = quaternion_msg[0] ** 2 + quaternion_msg[1] ** 2 + quaternion_msg[2] ** 2 + quaternion_msg[3] ** 2
    s = norm ** (-0.5)
    quaternion_msg[0] *= s
    quaternion_msg[1] *= s
    quaternion_msg[2] *= s
    quaternion_msg[3] *= s
    return quaternion_msg
