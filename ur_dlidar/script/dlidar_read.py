import rospy
import serial
from sensor_msgs.msg import Range

rospy.init_node("dlidar_read")
dlidar_pub = rospy.Publisher("/dlidar_data", Range, queue_size=1)
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
    data = ser.readlines(100)[-1].decode('ascii')
    strseg = data.split("\t")[:-1]
    if not len(strseg) in [8, 16]:
        print(data)
    else:
        if len(strseg) == 16:
            strseg = strseg[8:]
        for i, dis in enumerate(strseg):
            dis_n = float(dis)
            dlidar_msg.header.frame_id = 'dlidar{}'.format(i)
            if dis_n > 1000.0:
                dis_n = 1000
            dlidar_msg.range = dis_n/1000.0
            dlidar_msg.header.stamp = rospy.Time.now()

            dlidar_pub.publish(dlidar_msg)
            rospy.sleep(0.005)


