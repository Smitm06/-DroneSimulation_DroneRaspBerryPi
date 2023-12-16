#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State, AttitudeTarget
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

global des_ph, des_th, des_ps, des_throttle
des_ph = 0.0
des_th = 0.0
des_ps = 0.0
des_throttle = 0.0

global quad_state
quad_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

raspmode = "OFFBOARD"

def get_quaternion_from_euler(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


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


def determine_yaw():
    global quad_state, des_ps
    scale = ((Channels[3]-1500.0)/500.0)*5.0*5  #5.0 determines max speed in degrees/sec and set 10.0 as per the spped at which your script is running at
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

    att_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100) # 100 Hz

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    att = AttitudeTarget()

    while(not rospy.is_shutdown() ):

        # To get the desired values of the attitude and throttle
        des_ph = ((Channels[0] - 1500.0) / 500.0) * 45.0 * 3.14159265359 / 180.0
        des_th = ((Channels[1] - 1500.0) / 500.0) * 45.0 * 3.14159265359 / 180.0
        determine_yaw() 
        des_throttle = (Channels[2] - 1000.0) / 1000.0

        print(" des_ph -> " + str(des_ph) + " des_th -> " + str(des_th)+ " des_ps -> " + str(des_ps) + " des_throttle -> " + str(des_throttle))

        #  Assigning the values to the topics
        att.orientation = Quaternion(*quaternion_from_euler(des_ph,des_th,des_ps)) # The inputs are in radians
        att.thrust = des_throttle
        att.type_mask = 7

        # finally publish the data
        att_pub.publish(att)
        rate.sleep()
