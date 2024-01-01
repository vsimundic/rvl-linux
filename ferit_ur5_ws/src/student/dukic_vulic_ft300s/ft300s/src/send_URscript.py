#!/usr/bin/env python3
import socket
import time

HOST="192.168.22.14"
PORT=30002


print("Starting program")

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((HOST,PORT))
time.sleep(1)
f = open("/home/lukrecia/catkin_ws/src/ft300s/src/ur_script.txt", "rb")
l = f.read(1024)
print(l)
a=l.decode('utf-8')
a=a.split('\n')
print(a)
l=bytes(a[0],'utf-8')

print(l)

for i in range(len(a)-1):
    print(bytes(a[i]+'\n','utf-8'))
    s.sendall(bytes(a[i]+'\n','utf-8'))
    time.sleep(10)


# while (l):
#     s.sendall(l)
#     #time.sleep(2)
#     l = f.read(1024)
#     print(l)


print("End program")
data=s.recv(1024)
s.close()

print("Receved",repr(data))