#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from drone_pkg.msg import human_inputs
from drone_pkg.msg import quad_states

current_state = State()
raspmode = "STABILIZED" 
mode = "Position"  #Position,Velocity,Attitude 
error = 1.0
# Channel1 = 1500
# Channel2 = 1500
# Channel3 = 1500
# Channel4 = 1500
# Channel5 = 1500
# Channel6 = 1500
# Channel7 = 1500
# Channel8 = 1500
# Channel9 = 1500
# Channel10 = 1500
global Channels
Channels = [1500,1500,1500,1500,1500,1500,1500,1500,0,0]
global quad_state
quad_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   #///
#
Coordinates = [ 0, 0, 0, 0] #x,y,z,psi

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

def pos(Channels):
    pass

def att(Channels):
    pass

def vel(Channels):
    pass


def state_cb(msg):
    global current_state
    current_state = msg

def is_at_position(quad_state, x, y, z, offset):
    """offset: meters"""
    rospy.logdebug(
        "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
            quad_state[0], quad_state[1], quad_state[2]))

    desired = np.array((Coordinates[0], Coordinates[1], Coordinates[2]))
    pos = np.array((quad_state[0],
                    quad_state[1],
                    quad_state[2]))
    return np.linalg.norm(desired - pos) < offset

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

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100) # 100 Hz

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    desired_pose = PoseStamped()

    desired_pose.pose.position.x = Coordinates[0]
    desired_pose.pose.position.y = Coordinates[1]
    desired_pose.pose.position.z = Coordinates[2]

    # Send a few setpoints before starting
    # for i in range(500):
    #     if(rospy.is_shutdown()):
    #         break

    #     local_pos_pub.publish(desired_pose)
    #     rate.sleep()

    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = raspmode

    # arm_cmd = CommandBoolRequest()
    # arm_cmd.value = True

    last_req_mode = rospy.Time.now()
    # last_req_arm = rospy.Time.now()
    rospy.loginfo("Current armed state: " + str(current_state.armed) + "and Current mode: " +current_state.mode)
    # rospy.loginfo(rospy.Time.now())
    # rospy.loginfo(rospy.Duration(5.0))     it's in terms of sec and time will be in nanosec
    while(not rospy.is_shutdown()):
        if(current_state.mode != raspmode and Channels[7]>1500):
            if((rospy.Time.now() - last_req_mode) > rospy.Duration(5.0)):
                for i in range(500):
                        if(rospy.is_shutdown()):
                            break

                    local_pos_pub.publish(desired_pose)
                        
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    last_req_mode = rospy.Time.now()
                    if(current_state.mode == raspmode):
                        rospy.loginfo(raspmode + " enabled")
                    else:  
                        rospy.loginfo("unable to set " + raspmode + " Retrying")
        # if(not current_state.armed and ((rospy.Time.now() - last_req_arm) > rospy.Duration(5.0))):
        #     last_req_arm = rospy.Time.now()
        #     if(arming_client.call(arm_cmd).success == True):
        #         rospy.loginfo("Vehicle armed and Current mode: " +current_state.mode)
        #     else:
        #         rospy.loginfo("Vehicle not armed and Current mode: " +current_state.mode)
        while(not rospy.is_shutdown() and current_state.mode == raspmode and current_state.armed):
            rospy.loginfo("Ready to fly")
            # local_pos_pub.publish(desired_pose)

            

            if(mode == "Position"):
                quaternion = get_quaternion_from_euler(0, 0, math.radians(Coordinates[3]))
                rospy.loginfo(quaternion)
                timeout=60
                # set a position setpoint
                desired_pose.pose.position.x = Coordinates[0]
                desired_pose.position.y = Coordinates[1]
                desired_pose.position.z = Coordinates[2]
                local_pos_pub.publish(desired_pose)
                rospy.loginfo("attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".format(Coordinates[0], Coordinates[1], Coordinates[2], quad_state[0],quad_state[1],quad_state[2]))

                
                desired_pose.pose.orientation = Quaternion(*quaternion)

                # does it reach the position in 'timeout' seconds?
                reached = False
                for i in xrange(timeout * 100):
                    if drone.is_at_position(quad_state,Coordinates[0],Coordinates[1],Coordinates[2], error):
                        rospy.loginfo("position reached | seconds: {0} of {1}".format(i / 100, timeout))
                        local_pos_pub.publish(desired_pose)
                        reached = True
                        break

                    try:
                        rate.sleep()
                    except rospy.ROSException as e:
                        self.fail(e)

                if(reaches == False):
                    rospy.loginfo("took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".format(quad_state[0],quad_state[1],quad_state[2], timeout))
                        
                    elif(mode == "Velocity"):
                        pass
                    elif(mode == "Attitude"):
                        pass

            rate.sleep()
            

        
        
        rate.sleep()
