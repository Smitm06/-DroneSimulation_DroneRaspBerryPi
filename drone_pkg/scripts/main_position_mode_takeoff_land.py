#!/usr/bin/env python3

import rospy
import time
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from drone_pkg.msg import quad_states
from tf.transformations import quaternion_from_euler

global quad_state
quad_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

global current_coordinates, des_coordinates, error, check, raspmode, num_points, current_state

current_coordinates=[0.0,0.0,0.0,0.0]   #x,y,z and yaw(degrees)
des_coordinates=[
                 [0.0,0.0,0.0,0.0], #x,y,z and yaw(degrees)
                 [0.0,0.0,3.0,0.0],
            
                 [0.0,0.0,5.0,0.0],
                
                 [0.0,0.0,3.0,0.0],
               
               
                 [0.0,0.0,0.0,0.0]
                ]    #all points to be covered

num_points=5 #no. of des_coordinates
error = [1.0,1.0,1.0,5.0] #x,y,z and yaw(degrees)
check = False
complete = False
raspmode = "OFFBOARD"

def reached(i):
    global check,current_coordinates,des_coordinates
    diff_0 = current_coordinates[0]-des_coordinates[i][0]
    diff_1 = current_coordinates[1]-des_coordinates[i][1]
    diff_2 = current_coordinates[2]-des_coordinates[i][2]
    diff_3 = current_coordinates[3]-des_coordinates[i][3]

    if (abs(diff_0)<error[0] and (abs(diff_1)<error[1]) and (abs(diff_2)<error[2]) ):
        check = True
        print("true")


def state_cb(msg):
    global current_state
    current_state = msg

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


if __name__ == "__main__":

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    rospy.Subscriber("/quad/quad_states", quad_states, quad_states_callback)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100) # 100 Hz

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = False

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pos = PoseStamped()

    while(not rospy.is_shutdown() ):

        for x in range(num_points-1):
            check = False
            pos.pose.position.x = des_coordinates[x][0]
            pos.pose.position.y = des_coordinates[x][1]
            pos.pose.position.z = des_coordinates[x][2]  
            pos.pose.orientation = Quaternion(*quaternion_from_euler(0.0,0.0,des_coordinates[x][3]*3.14159265359/180.0))
            print(" x -> " + str(pos.pose.position.x) + " y -> " + str(pos.pose.position.y) + " z -> " + str(pos.pose.position.z)+ "yaw ->" + str(des_coordinates[x][3]))
            
            while(not rospy.is_shutdown() and current_state.mode != raspmode and not current_state.armed):
                local_pos_pub.publish(pos)
                print("Waiting")
                rate.sleep()

            while(not rospy.is_shutdown() and not check):
                current_coordinates[0]=quad_state[0]
                current_coordinates[1]=quad_state[1]
                current_coordinates[2]=quad_state[2]
                current_coordinates[3]=quad_state[5]

                local_pos_pub.publish(pos)
                reached(x)
                rate.sleep()

        print("I have completed the mission")

        time.sleep(5.0)
        for x in range(50):
            local_pos_pub.publish(pos)
        while(current_state.armed):
            if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle disarmed")


        complete=True






