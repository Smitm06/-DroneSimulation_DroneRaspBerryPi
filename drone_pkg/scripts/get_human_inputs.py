#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import RCIn
from drone_pkg.msg import human_inputs

global channels

def callback(data):
    global channels
    channels=human_inputs()
    channels.channel1=data.channels[0]
    channels.channel2=data.channels[1]
    channels.channel3=data.channels[2]
    channels.channel4=data.channels[3]
    channels.channel5=data.channels[4]
    channels.channel6=data.channels[5]
    channels.channel7=data.channels[6]
    channels.channel8=data.channels[7]
    pub=rospy.Publisher("/quad/human_inputs",human_inputs,queue_size=0)
    pub.publish(channels)



def listener():
    rospy.init_node('get_rc', anonymous=True)
    rospy.Subscriber("/mavros/rc/in", RCIn, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()