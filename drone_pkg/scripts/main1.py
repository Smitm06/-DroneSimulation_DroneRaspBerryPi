#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from drone_pkg.msg import quad_states
from nav_msgs.msg import Odometry

# global quad_position_local
# global quad_velocity_local

global quad_acceleration_local
global quad_atti_euler
global quad_angular_vel

# quad_position_local = [0.0, 0.0, 0.0]
# quad_velocity_local = [0.0, 0.0, 0.0]

quad_acceleration_local = [0.0, 0.0, 0.0]
quad_atti_euler     = [0.0, 0.0, 0.0]
quad_angular_vel     = [0.0, 0.0, 0.0]

# def callback_GPS(data):
#     global quad_position_local
#     quad_position_local = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
#     quad_velocity_local = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]

def callback(data):
    global quad_atti_euler
    global quad_angular_vel
    global quad_acceleration_local

    quad_atti_euler = quaternion_to_euler([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
    quad_angular_vel = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
    quad_acceleration_local = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
       
def listener():
    rospy.init_node('get_imu', anonymous=True)
    rospy.Subscriber("/mavros/imu/data", Imu, callback)
    # rospy.Subscriber("/mavros/global_position/local", Odometry, callback_GPS)

    quad_state_assign = quad_states()
    pub=rospy.Publisher("/quad/quad_states",quad_states,queue_size=0)
    # quad_state_assign.quad_x = quad_position_local[0]
    # quad_state_assign.quad_y = quad_position_local[1]
    # quad_state_assign.quad_z = quad_position_local[2]

    # quad_state_assign.quad_x_dot = quad_velocity_local[0]
    # quad_state_assign.quad_y_dot = quad_velocity_local[1]
    # quad_state_assign.quad_z_dot = quad_velocity_local[2]

    quad_state_assign.quad_ph_dot = quad_angular_vel[0]
    quad_state_assign.quad_th_dot = quad_angular_vel[1]
    quad_state_assign.quad_ps_dot = quad_angular_vel[2]

    quad_state_assign.quad_ph = quad_atti_euler[0]
    quad_state_assign.quad_th = quad_atti_euler[1]
    quad_state_assign.quad_ps = quad_atti_euler[2]

    quad_state_assign.quad_x_dot_dot = quad_acceleration_local[0]
    quad_state_assign.quad_y_dot_dot = quad_acceleration_local[1]
    quad_state_assign.quad_z_dot_dot = quad_acceleration_local[2]

    pub.publish(quad_state_assign)

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