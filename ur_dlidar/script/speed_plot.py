import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


def callback(msg):
    actual_speed = msg.velocity
    pub1.publish(actual_speed[0])
    pub2.publish(actual_speed[1])
    pub3.publish(actual_speed[2])
    pub4.publish(actual_speed[3])
    pub5.publish(actual_speed[4])
    pub6.publish(actual_speed[5])


if __name__ == "__main__":
    rospy.init_node("speed_plot_node", anonymous=True)
    pub1 = rospy.Publisher("/speed/shoulder_pan_joint", Float64, queue_size=1)
    pub2 = rospy.Publisher("/speed/shoulder_lift_joint", Float64, queue_size=1)
    pub3 = rospy.Publisher("/speed/elbow_joint", Float64, queue_size=1)
    pub4 = rospy.Publisher("/speed/wrist_1_joint", Float64, queue_size=1)
    pub5 = rospy.Publisher("/speed/wrist_2_joint", Float64, queue_size=1)
    pub6 = rospy.Publisher("/speed/wrist_3_joint", Float64, queue_size=1)
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()