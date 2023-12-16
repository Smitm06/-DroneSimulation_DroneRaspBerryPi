#!/usr/bin/env python3

import rospy
import numpy as np


from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from drone_pkg.msg import human_inputs
from drone_pkg.msg import quad_states


global current_state
current_state = State()
global Channels
Channels = [1500,1500,1500,1500,1500,1500,1500,1500,0,0]
# channel - 1 = y
# channel - 2 = x
# channel - 3 = z
# channel - 4 = yaw 

# channel - 5 = Mode switch [Loiter, loiter, stabilize]
# channel - 6 = empty
# channel - 7 = empty
# channel - 8 = toggle offboard

global des_x, des_y, des_z
des_x = 0.0
des_y = 0.0
des_z = 0.0

global quad_state
quad_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

raspmode = "OFFBOARD"



def state_cb(msg):
    global current_state
    current_state = msg

def channels_callback(data):
    global Channels
    Channels[0]=data.channel1
    Channels[1]=data.channel2
    Channels[2]=data.channel3
    Channels[3]=data.channel4
    Channels[4]=data.channel5
    Channels[5]=data.channel6
    Channels[6]=data.channel7
    Channels[7]=data.channel8

def quad_states_callback(data):
    global quad_state
    quad_state[0]=data.quad_x
    quad_state[1]=data.quad_y
    quad_state[2]=data.quad_z
    quad_state[3]=data.quad_ph
    quad_state[4]=data.quad_th
    quad_state[5]=data.quad_ps
    quad_state[6]=data.quad_x_dot
    quad_state[7]=data.quad_y_dot
    quad_state[8]=data.quad_z_dot
    quad_state[9]=data.quad_ph_dot
    quad_state[10]=data.quad_th_dot
    quad_state[11]=data.quad_ps_dot
    quad_state[12]=data.quad_x_dot_dot
    quad_state[13]=data.quad_y_dot_dot
    quad_state[14]=data.quad_z_dot_dot




if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    rospy.Subscriber("/quad/quad_states", quad_states, quad_states_callback)
    rospy.Subscriber("/quad/human_inputs", human_inputs, channels_callback)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100) # 100 Hz

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    vel = PositionTarget()

    while(not rospy.is_shutdown() ):

        
        des_x = ((Channels[0] - 1500.0) / 500.0) 
        des_y = ((Channels[1] - 1500.0) / 500.0)
        des_z = ((Channels[2] - 1500.0) / 500.0)

        print(" des_x -> " + str(des_x) + " des_y -> " + str(des_y) + " des_z -> " + str(des_z))

        #  Assigning the values to the topics
        vel.coordinate_frame=8
        vel.type_mask=3527
        vel.velocity.x=des_x
        vel.velocity.y=des_y
        vel.velocity.z=des_z

        # finally publish the data
        vel_pub.publish(vel)
        rate.sleep()
