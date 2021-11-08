#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import WrenchStamped
import socket
from time import gmtime, strftime
import csv
import ConfigParser
import time

def make_wrench_stamped(force, torque):
    '''
    Make a WrenchStamped message without all the fuss Frame defaults to body
    '''
    wrench = WrenchStamped()
    wrench.header.frame_id = "FT_link"
    wrench.header.stamp = rospy.Time(0)
    wrench.wrench.force.x = force[0]
    wrench.wrench.force.y = force[1]
    wrench.wrench.force.z = force[2]
    wrench.wrench.torque.x = torque[0]
    wrench.wrench.torque.y = torque[1]
    wrench.wrench.torque.z = torque[2]
    return wrench

class socket_connection ():

    def __init__ (self, publisher_object, rate):

        #config = ConfigParser.ConfigParser()
        #config.read("config.ini")

        #self.host_ip = config.get('Information', 'ip_address_ur3')
        #self.port = int (config.get ('Information', 'port'))
        self.host_ip = "192.168.56.2"
        self.port = 63351
        self.socket_object = socket.create_connection((self.host_ip, self.port), timeout=0.5)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False


    def connect (self):

        try:

            print("Warning: Connecting to ur3 ip adress: " + self.host_ip)
            self.socket_object.connect((self.host_ip, self.port))

            try:
                while (1):

                    deneme = self.socket_object.recv(1024)
                    denemeFx = float(deneme.split(',')[0].split('(')[1])
                    denemeFy = float(deneme.split(',')[1])
                    denemeFz = float(deneme.split(',')[2])
                    denemeMx = float(deneme.split(',')[3])
                    denemeMy = float(deneme.split(',')[4])
                    denemeMz = float(deneme.split(',')[5].split(')')[0])
                    ForceData = [denemeFx, denemeFy, denemeFz]
                    TorqueData = [denemeMx, denemeMy, denemeMz]
                    rospy.loginfo(ForceData)
                    #rospy.loginfo(TorqueData)
                    wrench = make_wrench_stamped(ForceData, TorqueData)
                    self.publisher_object.publish(wrench)
                    self.publisher_rate.sleep()
                  
            except KeyboardInterrupt:

                self.socket_object.close()

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False



def main():
    
    pub = rospy.Publisher('/robotiq_ft_wrench', WrenchStamped, queue_size=10)
    rospy.init_node('ft_sensor', anonymous=True)
        
    rate = rospy.Rate(10) 

    socket_connection_obj = socket_connection(pub, rate)    
    socket_connection_obj.connect()
          

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass                         

    
                        
