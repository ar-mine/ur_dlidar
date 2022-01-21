import rospy
import serial
from sensor_msgs.msg import Range
import numpy as np

rospy.init_node("dlidar_read")

dlidar_pub = [None] * 8
dlidar_msg = Range()
dlidar_msg.max_range = 1.0
dlidar_msg.min_range = 0.02
dlidar_msg.field_of_view = 0.536
dlidar_msg.radiation_type = 0

# Serial takes two parameters: serial device and baudrate
ser = serial.Serial('/dev/ttyACM0', 9600)
if ser.isOpen():
    print("open success")
else:
    print("open failed")
    exit(-1)


def on_shutdown():
    ser.close()


while not rospy.is_shutdown():
    data = ser.readline().decode().strip()

    if 'Connection' in data or 'Connected' in data:
        rospy.logwarn(data)
        continue
    strseg = np.array([float(s) for s in data.split('\t')])

    for i, dis in enumerate(strseg):
        dlidar_msg.header.frame_id = 'dlidar{}'.format(i)
        if dis > 1000.0 or dis == 0:
            dis = 1000
        dlidar_msg.range = dis / 1000.0
        dlidar_msg.header.stamp = rospy.Time.now()
        if dlidar_pub[i] is None:
            dlidar_pub[i] = rospy.Publisher('/dlidar_data'+str(i), Range, queue_size=1)

        dlidar_pub[i].publish(dlidar_msg)
