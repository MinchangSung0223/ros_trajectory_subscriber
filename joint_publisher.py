#!/usr/bin/python
import sys
import rospy, math, time
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



joint_trajectory_list = np.array([[-0.392699,-0.785085,0,-2.35559,0,1.57,0],
[-1.06065,2.2987,-0.343622,-2.69798,0.69044,0.45334,-0.425342],
[-0.276127,0.89656,-0.184268,-2.90509,-0.0852484,-2.14459,2.7548],
[0.699951,-1.4898,-1.64341,-0.196771,-1.8963,0.0324484,2.95919],
[2.62631,-1.7628,-1.48536,-1.62642,-1.76757,1.47608,1.87821]],dtype=float)

if __name__ == "__main__":
	pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
	rospy.init_node('joint_trajectory_pub')
	rate = rospy.Rate(10)
	if len(sys.argv) <8:
		print("ERROR:Please Input joint values ")
		exit(0);


	jt = JointTrajectory()
	jt.header.stamp = rospy.Time.now()
	jt.joint_names.append("panda_joint1" )
	jt.joint_names.append("panda_joint2" )
	jt.joint_names.append("panda_joint3" )
	jt.joint_names.append("panda_joint4" )
	jt.joint_names.append("panda_joint5" )
	jt.joint_names.append("panda_joint6" )
	jt.joint_names.append("panda_joint7" )
	N = 30;
	for i in range(0,len(joint_trajectory_list)):
		
		points = JointTrajectoryPoint();
		temp = joint_trajectory_list[i]
		points.positions.append(float(temp[0]));
		points.positions.append(float(temp[1]));
		points.positions.append(float(temp[2]));
		points.positions.append(float(temp[3]));
		points.positions.append(float(temp[4]));
		points.positions.append(float(temp[5]));
		points.positions.append(float(temp[6]));
		points.time_from_start =  rospy.Duration.from_sec(0.01);
		jt.points.append(points)
	rospy.loginfo(points)
	pub.publish(jt)
	rate.sleep()
	pub.publish(jt)
	rate.sleep()
	pub.publish(jt)
	rate.sleep()
