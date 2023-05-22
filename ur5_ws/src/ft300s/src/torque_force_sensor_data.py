#!/usr/bin/env python3
# license removed for brevity
import rospy
##from std_msgs.msg import String
from std_msgs.msg import Float32
import socket
from time import gmtime, strftime
#import ConfigParser
import time
import struct
import sys


class socket_connection ():

    def __init__ (self, publisher_object, rate):

        try:

            #config = ConfigParser.ConfigParser()
            #config.read("config.ini")

            self.host_ip = "192.168.22.14"
            self.port = 63351
            self.output_file_location = "src/ft300s/"
            
            self.write_object = True

            #if self.output_file_location is "":
            #    self.output_file_location = ""
            #elif self.output_file_location is "None":
            #    self.write_object = False
            #else:
            #    self.output_file_location = self.output_file_location + "/" 

                
        except:

            print ("Warning: Config.ini file problem, please check Config.ini")

        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False


    def connect (self):

        try:

            print("Warning3: Connecting to Ur5 ip adress: " + self.host_ip)
            self.socket_object.connect(( self.host_ip, self.port ))
            print(self.write_object)    
            if self.write_object is True:
                print("Warning: File Write Location: " + self.output_file_location)
                #f = open (self.output_file_location + "DataStream.csv", "w")
                       
            try:
                print("Writing nothing in DataStream.csv, Press ctrl + z to stop")
                while (1):
                    #print("writing....")
                    deneme = self.socket_object.recv(1024) #procita bajte
                    #print(deneme)
                    #print(deneme[1: -1])
                    #print(type(deneme))
                    #print(sys.getsizeof(deneme))
                    
                    bytes_unpacked_deneme = deneme.decode('utf-8')[1:-1]
                    #print(bytes_unpacked_deneme)
                    denemeF = bytes_unpacked_deneme.split(" , ")
                    print(str(denemeF))
                    #print(type(denemeF))
                    print("\n")

                    #force_torque_values = (self.socket_object.recv(1024).replace("(","*")).replace(")","*")
                    
                    #value_list = force_torque_values.split("*")
                   
                    
                    #rospy.loginfo(value_list [1])
                    ##rospy.loginfo(denemeF)



                    #self.publisher_object.publish(value_list[1])
                    self.publisher_object.publish(denemeF)
                    ##self.publisher_rate.sleep()
                

                    #if self.write_object is True:       
                    #    f.write(strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime()) + ": " +force_torque_values)
                                              
            except KeyboardInterrupt:
                        
                f.close
                self.socket_object.close

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False



def main():
    

##    pub = rospy.Publisher('ft300_force_torque', String, queue_size=10)
    
    pub = rospy.Publisher('ft300_force_torque', Float32, queue_size=10)    
    rospy.init_node('torque_force_sensor_data', anonymous=True)
        
    rate = rospy.Rate(10) 

    socket_connection_obj = socket_connection(pub, rate)    
    socket_connection_obj.connect()
          

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass                         

    
                        
