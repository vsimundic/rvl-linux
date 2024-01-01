#!/usr/bin/env python3
import rtde_control
import rtde_receive
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.22.14")
actual_q = rtde_r.getActualQ()
print(actual_q)
rtde_c = rtde_control.RTDEControlInterface("192.168.22.14")
rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)
actual_q = rtde_r.getActualQ()
print(actual_q)




#radi  za primanje podataka
# import rtde_receive
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.22.14")
# actual_q = rtde_r.getActualQ()
# print(actual_q)