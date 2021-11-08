#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import WrenchStamped
import socket
import time

class socket_connection:
    def __init__(self, publisher_object, rate):
        self.host_ip = "192.168.56.2"
        self.port = 63351
        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False

    def connect(self):

        try:

            print("Warning: Connecting to UR3 ip adress: " + self.host_ip)
            self.socket_object.connect((self.host_ip, self.port))

            try:
                while 1:
                    force_torque_values = (self.socket_object.recv(1024).replace(b"(", b"*")).replace(b")", b"*")

                    value_list = force_torque_values.split(b"*")
                    rospy.loginfo(value_list[1])

                    values = value_list[1].split(b",", 6)
                    msg = WrenchStamped()

                    msg.header.frame_id = "FT_link"
                    msg.header.stamp = rospy.Time.now()
                    msg.wrench.force.x = float(values[0])
                    msg.wrench.force.y = float(values[1])
                    msg.wrench.force.z = float(values[2])

                    msg.wrench.torque.x = float(values[3])
                    msg.wrench.torque.y = float(values[4])
                    msg.wrench.torque.z = float(values[5])

                    self.publisher_object.publish(msg)
                    self.publisher_rate.sleep()

            except KeyboardInterrupt:
                self.socket_object.close()

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False


def main():
    pub = rospy.Publisher('robotiq_ft_wrench', WrenchStamped, queue_size=10)
    rospy.init_node('ft_sensor_data', anonymous=True)

    rate = rospy.Rate(125)

    socket_connection_obj = socket_connection(pub, rate)
    socket_connection_obj.connect()


if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass
