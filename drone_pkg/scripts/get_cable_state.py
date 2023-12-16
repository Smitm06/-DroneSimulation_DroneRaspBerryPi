#!/usr/bin/env python3
import serial
import rospy
import time
from drone_pkg.msg import cable_states

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.reset_input_buffer()
    rospy.init_node('get_cam', anonymous=True)
    pub=rospy.Publisher("/quad/cable_states",cable_states,queue_size=0)
    rate = rospy.Rate(100) # 100 Hz

    while True:
        line1 = ser.readline().decode('utf-8').rstrip()
        line = cable_states()
        if(len(line1)==13 and line1[6]=='_' and line1[12]=='/'):
            line2 = line1.split('_')
    
            cable_ph1 =line2[0]
            cable_ph2 = str(cable_ph1[1:6])

            cable_ph  = int(cable_ph2) - 50000
            
            cable_th1 =line2[1]
            cable_th2 = str(cable_th1[0:5])
            cable_th  = int(cable_th2) - 50000
            line.cable_ph = cable_ph/100.0
            line.cable_th = cable_th/100.0
            pub.publish(line)
            # print(" cable_ph -> " + str(cable_ph) + " cable_th -> " + str(cable_th) )
        
        try:
            rate.sleep()
        except rospy.ROSException as e:
            print(e)

        rate.sleep()