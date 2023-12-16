#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from drone_pkg.msg import human_inputs
from drone_pkg.msg import quad_states
from tf.transformations import quaternion_from_euler

global current_state
current_state = State()
global Channels
Channels = [1500,1500,1500,1500,1500,1500,1500,1500,0,0]
# channel - 1 = roll angle
# channel - 2 = pitch angle
# channel - 3 = throttle angle
# channel - 4 = yaw angle

# channel - 5 = Mode switch [Loiter, loiter, stabilize]
# channel - 6 = empty
# channel - 7 = empty
# channel - 8 = toggle offboard

global des_ph, des_th, des_ps #in radians
des_ph = 0.0
des_th = 0.0
des_ps = 0.0

global quad_state
quad_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

global coordinates
coordinates=[0,0,0] #(x,y,z)(forward,right,down)

global boxlength #(equals 1/2 the total length,width and height)
boxlength = 1.0

offsets = [0.0,0.0,0.1]

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
    quad_state[5]=data.quad_ps   #yaw
    quad_state[6]=data.quad_x_dot
    quad_state[7]=data.quad_y_dot
    quad_state[8]=data.quad_z_dot
    quad_state[9]=data.quad_ph_dot
    quad_state[10]=data.quad_th_dot
    quad_state[11]=data.quad_ps_dot
    quad_state[12]=data.quad_x_dot_dot
    quad_state[13]=data.quad_y_dot_dot
    quad_state[14]=data.quad_z_dot_dot

def determine_yaw():
    global quad_state, des_ps
    scale = ((Channels[3]-1500.0)/500.0)*5.0  #5.0 determines max speed in degrees/sec and set 10.0 as per the spped at which your script is running at
    des_ps_deg= (quad_state[5])+scale

    if(des_ps_deg>=360.0):
        des_ps_deg= des_ps_deg-360
    elif(des_ps_deg<0):
        des_ps_deg= des_ps_deg+360

    des_ps= des_ps_deg* 3.14159265359 / 180.0



if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    rospy.Subscriber("/quad/quad_states", quad_states, quad_states_callback)
    rospy.Subscriber("/quad/human_inputs", human_inputs, channels_callback)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100) # 100 Hz

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pos = PoseStamped()

    while(not rospy.is_shutdown() ):
        determine_yaw()

        coordinates[0]= ((Channels[1] - 1500.0)/500.0)* boxlength
        coordinates[1]= ((Channels[0] - 1500.0)/500.0)* boxlength
        coordinates[2]= ((Channels[2]-1000.0)/500.0)* boxlength
        pos.pose.position.x = coordinates[0]+offsets[0]
        pos.pose.position.y = coordinates[1]+offsets[1]
        pos.pose.position.z = coordinates[2]+offsets[2]
       
        pos.pose.orientation = Quaternion(*quaternion_from_euler(des_ph,des_th,des_ps)) # The inputs are in radians
        print(" x -> " + str(pos.pose.position.x) + " y -> " + str(pos.pose.position.y) + " des_throttle -> " + str(pos.pose.position.z) + " yaw -> "+ str(des_ps*180/3.14159265359))
        #  Assigning the values to the topics
        # finally publish the data
        local_pos_pub.publish(pos)
        rate.sleep()
