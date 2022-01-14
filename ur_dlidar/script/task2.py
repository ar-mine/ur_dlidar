import rospy

# change
from admittance_control import AdmittanceControl

if __name__ == "__main__":
    rospy.init_node("Task2")
    test = AdmittanceControl()
