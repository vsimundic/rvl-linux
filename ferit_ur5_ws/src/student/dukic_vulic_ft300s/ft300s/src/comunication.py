#!/usr/bin/env python3
import socket
import time

HOST="192.168.22.14"
PORT=30002

print("Starting program")

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((HOST,PORT))
time.sleep(0.5)

print("Set output 1 and 2 high")

s.sendall(("set_digital_out(1,True)"+"\n").encode('utf-8'))
time.sleep(0.1)
s.sendall(("set_digital_out(2,True)"+"\n").encode('utf-8'))
time.sleep(2)

print("Robot moves position")


s.sendall(("movel(p[0, -0.561, 0.5, 1.52, -0.57, 0.5], a=1.0, v=0.05)" + "\n").encode('utf-8')) 
# s.sendall(("thread Force_properties_calculation_thread_1():"+"\n").encode('utf-8'))
# s.sendall(("  while (True):"+"\n").encode('utf-8'))
# s.sendall(("force_mode(base_pose(),[1,1,0,0,0,0],[0,0,0,0,0,0],2,[0.15,0.15,0.1,0.35,0.35,0.35])"+"\n").encode('utf-8'))
# s.sendall(("sync()"+"\n").encode('utf-8'))
# s.sendall(("end"+"\n").encode('utf-8'))
# s.sendall(("end"+"\n").encode('utf-8'))
time.sleep(10)

s.sendall(("set_digital_out(1,False)"+"\n").encode('utf-8'))
time.sleep(0.1)
s.sendall(("set_digital_out(2,False)"+"\n").encode('utf-8'))
time.sleep(0.1)

data=s.recv(1024)
s.close()

print("Receved",repr(data))