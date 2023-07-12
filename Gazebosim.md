Gazebo version - Gazebo11

Aim Of Simulation: The aim is to simulate our real drone in real time in gazebo, that is , what ever our real drone is doing ( roll, pitch, up , down etc) , the simulator will mimic in real time. To acchieve this, we will give position and orientation data from GPS and imu to our gazebo as input in real time and gazebo will publish this to virtual drone. So by this in real time we can mimic the real drone in gazebo. So gazebo will take 6 things in input which are x,y,z and roll,pitch,yaw angles. 

First step is to import our model to gazebo environment and next step is to control it to simulate it.
So to import an model to Gazebo, there required two files, one is urdf file which has details of model structure and other is launch file which launches it to gazebo.
URDF (Unified Robot Description Format) is an XML-based file format used in the Robot Operating System (ROS) to describe the structure and visual properties of a robot model. It has links , joints , visuals and collisions. 
Example of basic drone urdf file is below:
```
 <?xml version="1.0"?>
   <robot name="drone">

     <link name="base_link">
       <visual>
		 <origin xyz="0 0 0" rpy="0 0 0" />
         <geometry>
           <box size="0.4 0.2 0.04"/>
         </geometry>
       </visual>
       <collision>
		  <origin xyz="0 0 0" rpy="0 0 0" />
          <geometry>
            <box size="0.4 0.04 0.04"/>
          </geometry>
        </collision>
        <!--<inertial>
           <mass value="0.3"/>
           <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
         </inertial>-->
     </link>

     <link name="FL">
       <visual>
		 <origin xyz="0.16 0.13 0" rpy="0 0 0" />
         <geometry>
            <cylinder length="0.03" radius="0.13"/>
         </geometry>
       </visual>
       <collision>
          <origin xyz="0.16 0.13 0" rpy="0 0 0" />
          <geometry>
            <cylinder length="0.03" radius="0.13"/>
          </geometry>
        </collision>
        <inertial>
           <mass value="0.2"/>
           <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
         </inertial>
     </link>
     <link name="FR">
       <visual>
		 <origin xyz="0.16 -0.13 0" rpy="0 0 0" />
         <geometry>
            <cylinder length="0.03" radius="0.13"/>
         </geometry>
       </visual>
       <collision>
          <origin xyz="0.16 -0.13 0" rpy="0 0 0" />
          <geometry>
            <cylinder length="0.03" radius="0.13"/>
          </geometry>
        </collision>
        <inertial>
           <mass value="0.2"/>
           <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
         </inertial>
     </link>
     <link name="BL">
       <visual>
		 <origin xyz="-0.09 0.13 0" rpy="0 0 0" />
         <geometry>
            <cylinder length="0.03" radius="0.13"/>
         </geometry>
       </visual>
       <collision>
          <origin xyz="-0.09 0.13 0" rpy="0 0 0" />
          <geometry>
            <cylinder length="0.03" radius="0.13"/>
          </geometry>
        </collision>
        <inertial>
           <mass value="0.2"/>
           <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
         </inertial>
     </link>
     <link name="BR">
       <visual>
		 <origin xyz="-0.09 -0.13 0" rpy="0 0 0" />
         <geometry>
            <cylinder length="0.03" radius="0.13"/>
         </geometry>
       </visual>
       <collision>
          <origin xyz="-0.09 -0.13 0" rpy="0 0 0" />
          <geometry>
            <cylinder length="0.03" radius="0.13"/>
          </geometry>
        </collision>
        <inertial>
           <mass value="0.2"/>
           <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
         </inertial>
     </link>

     <joint name="base_to_FL" type="fixed">
       <parent link="base_link"/>
       <child link="FL"/>
     </joint>
     <joint name="base_to_FR" type="fixed">
       <parent link="base_link"/>
       <child link="FR"/>
     </joint>
     <joint name="base_to_BL" type="fixed">
       <parent link="base_link"/>
       <child link="BL"/>
     </joint>
     <joint name="base_to_BR" type="fixed">
       <parent link="base_link"/>
       <child link="BR"/>
     </joint>


     <gazebo reference="base_link">
       <material>Gazebo/Blue</material>
       <turnGravityOff>true</turnGravityOff>
     </gazebo>
   </robot>
```
   Next is the launch file below fo it.

```
?xml version="1.0" encoding="UTF-8"?>
<launch>
  
  <!-- Load the drone URDF -->
  <param name="robot_description" textfile="$(find drone_pose)/urdf/drone.urdf" />

  <!-- Spawn the drone in Gazebo -->
  <node name="spawn_drone" pkg="gazebo_ros" type="spawn_model" output="screen"
    args="-urdf -param robot_description -model drone1
          -x 0 -y 0 -z 0
          -R 0 -P 0 -Y 0" />
	  </launch>
```

   
   So this line (-model drone1) is assigning name to this drone that is drone1. Also we can give the coordinates and orientation for where to launch in gazebo envirnment.
   This line (find drone_pose)/urdf/drone.urdf" />) is the location of our urdf file. So in the workspae "my_workspace", in src , there is a package named "drone_pose" where there is a folder of urdf where .urdf file is there. So this launch file is accessing that urdf model and then launching it to gazebo environment .
    Now for its implementation, run `roscore` in one terminal. In other terminal, open empty world gazebo by this command `roslaunch gazebo_ros emptyworld.launch` .Then in next terminal launch our spawn file by the command `roslaunch drone_pose spawn.launch`. But before this command there two thing to be done that is in the terminal `cd my_workspace` run `catkin_make`. Then `source devel/setup.bash` to source the files.
    Also to launch two models simultaneously in gazebo we have to add one more model to our launch file. Below is the launch file for the same:
   ``` <?xml version="1.0" encoding="UTF-8"?>
<launch>
  
  <!-- Load the drone URDF -->
  <param name="robot_description" textfile="$(find drone_pose)/urdf/drone.urdf" />

  <!-- Spawn the drone in Gazebo -->
  <node name="spawn_drone" pkg="gazebo_ros" type="spawn_model" output="screen"
    args="-urdf -param robot_description -model drone1
          -x 0 -y 0 -z 0
          -R 0 -P 0 -Y 0" />
    <node name="spawn_drone_2" pkg="gazebo_ros" type="spawn_model" output="screen"
    args="-urdf -param robot_description -model drone2
          -x 5 -y 0 -z 0
          -R 0 -P 0 -Y 0" />      
</launch>
```
So for the second drone, we assign model name - drone2 and placed it to (5,0,0).
There is another thing like urdf, that is .sdf file. This is usually used when we import an cad model of .dae file type to our gazebo. 
In this project I had taken an model package from gazebo in built models which is parrot_bebop_2 which is a parrot(small) drone. So I copy pasted this pkg from home/.gazebo/models to drone_pose. In this pkg , there are two files that is .sdf and .config and one folde named "meshes" which contain .dae files of our model. 
model.sdf file: 
```
<?xml version="1.0"?>
<sdf version="1.5">
  <model name="parrot_bebop_2">
    <static>false</static>
    <pose>0 0 0.062656 0 0 0</pose>

    <!-- Main link -->
    <link name="body">
      <inertial>
        <mass>0.503</mass> <!-- kg -->
        <inertia> <!-- kg.m^2 -->
          <ixx>1.805e-3</ixx>
          <ixy>-8.619e-7</ixy>
          <iyy>1.764e-3</iyy>
          <ixz>1.555e-5</ixz>
          <iyz>3.595e-7</iyz>
          <izz>3.328e-3</izz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
      </inertial>
      <collision name="collision">
        <pose>-0.00424 0.00014 -0.00595 0 0 0</pose>
        <geometry>
          <box><size>0.316618 0.330564 0.089582</size></box>
        </geometry>
        <max_contacts>10</max_contacts>
        <surface>
          <bounce>
            <restitution_coefficient>0.828</restitution_coefficient>
            <threshold>0</threshold>
          </bounce>
          <friction>
            <ode>
              <mu>10</mu>
              <mu2>5</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000.0</kp>
              <kd>10.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="hull">
        <geometry>
          <mesh>
              <uri>model://parrot_bebop_2/meshes/hull.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="propeller_rr">
        <pose>-0.09035 -0.11423 -0.00501 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://parrot_bebop_2/meshes/propeller_rr.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="propeller_rl">
        <pose>-0.09035 0.11452 -0.00501 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://parrot_bebop_2/meshes/propeller_rl.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="propeller_fr">
        <pose>0.08456 -0.11435 -0.00501 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://parrot_bebop_2/meshes/propeller_fr.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="propeller_fl">
        <pose>0.08456 0.11463 -0.00501 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://parrot_bebop_2/meshes/propeller_fl.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

model.config file is below:

```
<?xml version="1.0"?>
<model>
    <name>Parrot Bebop 2</name>
    <version>1.0</version>
    <sdf version="1.5">model.sdf</sdf>

    <author>
      <name>Olivier Crave</name>
      <email>olivier.crave@parrot.com</email>
    </author>

    <description>
      A model of the Parrot Bebop 2 drone.
    </description>
</model>
```
Now for sdf model, there is different launch file named parrot.launch , which is below:

```
<launch>
  <!-- Start Gazebo with an empty world -->
  <arg name="world" default="$(find gazebo_ros)/worlds/empty.world"/>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world)"/>
  </include>

  <!-- Load the SDF model -->
  <arg name="sdf" default="$(find drone_pose)/parrot_bebop_2/model.sdf"/>
  <param name="robot_description" textfile="$(arg sdf)"/>

  <!-- Spawn the robot in Gazebo -->
  <node name="spawn_sdf" pkg="gazebo_ros" type="spawn_model" output="screen"
    args="-sdf -param robot_description -model drone1"/>
  <node name="spawn_sdf2" pkg="gazebo_ros" type="spawn_model" output="screen"
    args="-sdf -param robot_description -model drone2"/>

</launch>
```
By launching it same as previous command, the model is launched in our gazebo environment. 

So till now our drone model is launched in our gazebo.
Now the second phase begins that is to control it or to move it. So there are two ways we can do it. First is gazebo plugins, and other is gazebo API. 
So initially while learning gazebo and ros, I controlled two wheeler robot/car by gazebo plugin "differntial_drive_controller".
```    
<gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
      <alwaysOn>true</alwaysOn>
      <updateRate>20</updateRate>
      <leftJoint>joint_left_wheel</leftJoint>
      <rightJoint>joint_right_wheel</rightJoint>
      <wheelSeparation>0.4</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <torque>0.1</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>link_chassis</robotBaseFrame>
    </plugin>
  </gazebo>
</robot>
```
So what happens is we write a python script to make a node where instructions to change the velocity are return. This node subscribes to cmd_vel topic. Now this topic must subcribe to gazebo. For that plugin are there . Plugin link cmd_vel to gazebo, that is whatever message recieved by cmd_vel it gives to gazebo and the car gains the velocity accordingly. This line in above code (<commandTopic>cmd_vel</commandTopic>) is saying that cmd_vel must subscribe to this plugin. In this way a beautiful rqt graph forms where a controller node arrows to topic cmd_vel and it arrows to gazebo.
This plugin code above is written under urdf file of model. 
We can check if message is delivered to topic cmd_vel by command `rostopic echo /topic_name` .This command displays the messages published on a specific topic in real-time. It continuously prints the content of the messages as they are published.
rostopic info: This command provides information about a specific topic, including its message type, publishers, and subscribers. Command - `rostopic info /topic_name`

Now, coming to drone, we want to publish postion to gazebo, not velocity. For that no specific plugin availble. So one way is to build a plugin from scratch which is very difficult tedious task. So another way we can use is Gazebo API. The Gazebo API (Application Programming Interface) refers to the set of functions, classes, and methods provided by Gazebo, an open-source physics-based simulation platform. The Gazebo API allows developers to interact with Gazebo's core functionality, such as creating and manipulating simulation worlds, models, sensors, physics properties, and more.
Basically it is a custom plugin.
To install gazebo api in my ros noetic, I used chatgpt.
After installing, we can create an controller node which can directly subcribes to gazebo model states.
Below is the python script of the node.

```
#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import math
import time

if __name__ == '__main__':
   rospy.init_node('drone_controller')

   # Create a publisher to publish the model state
   model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
   rate = rospy.Rate(100)  # Publish at a rate of 10 Hz

   quad1 = ModelState()
   quad1.model_name = "drone1"

   quad2 = ModelState()
   quad2.model_name = "drone2"
   quad2.pose.position.x = 0
   quad2.pose.position.z = 1

   # Convert roll angle from degrees to radians
   roll_degrees = 30
   roll_radians = math.radians(roll_degrees)

   # Set the roll angle for quad2
   quad2.pose.orientation.x = math.sin(roll_radians / 2)
   quad2.pose.orientation.y = 0
   quad2.pose.orientation.z = 0
   quad2.pose.orientation.w= math.cos(roll_radians / 2)

   while not rospy.is_shutdown():
       # Publish the model state to Gazebo
       quad1.pose.position.x = 5*math.sin(2*time.time())
       quad2.pose.position.y = 2*math.sin(2*time.time())
       model_state_pub.publish(quad1)
       model_state_pub.publish(quad2)

       # Spin the ROS node
       rate.sleep()
```

  This line(model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)) is sayinf that thin node is publishing to gazebo 's set_model_state. `quad1.pose.position.x = 5*math.sin(2*time.time())` So here x position of drone1 is varying like sin function with amplitude 5. So it seems like drone 1 is moving but actually its publishing position at the rate of 100. That in 1 second it is publishing msg 100 times. so smooth. 

  
