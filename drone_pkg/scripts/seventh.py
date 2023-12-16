#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from drone_pkg.msg import quad_states

global euler

def callback(data):
    global euler
    # rospy.loginfo(data.orientation.x)
    qua_w=data.orientation.w
    qua_x=data.orientation.x
    qua_y=data.orientation.y
    qua_z=data.orientation.z
    euler= quaternion_to_euler([qua_w, qua_x, qua_y, qua_z])
    quad_state_assign = quad_states()
    pub=rospy.Publisher("/quad/quad_states",quad_states,queue_size=0)
    quad_state_assign.quad_ph = euler[0]
    quad_state_assign.quad_th = euler[1]
    quad_state_assign.quad_ps = euler[2]
    quad_state_assign.quad_ph_dot = data.angular_velocity.x
    quad_state_assign.quad_th_dot = data.angular_velocity.y
    quad_state_assign.quad_ps_dot = data.angular_velocity.z    
    quad_state_assign.quad_x_dot_dot = data.linear_acceleration.x
    quad_state_assign.quad_y_dot_dot = data.linear_acceleration.y
    quad_state_assign.quad_z_dot_dot = data.linear_acceleration.z
    pub.publish(quad_state_assign)
       
def listener():
    rospy.init_node('get_imu', anonymous=True)
    rospy.Subscriber("/mavros/imu/data", Imu, callback)
    rospy.spin()

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    Quaternion format: [w, x, y, z]
    Euler angles format: [roll, pitch, yaw]
    """
    # Extract quaternion components
    w, x, y, z = quaternion

    # Convert quaternion to Euler angles
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # Convert angles from radians to degrees
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    return [roll_deg, pitch_deg, yaw_deg]


if __name__ == '__main__':
    listener()