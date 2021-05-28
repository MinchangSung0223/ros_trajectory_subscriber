#!/usr/bin/python
import sys
import rospy, math, time
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



joint_trajectory_list = np.array([[-0.392699,-0.785085,0,-2.35559,0,1.57,0],
[2.03786,0.515102,-1.73876,-3.06448,-2.45146,0.963981,0.929993],
[0.924225,1.8232,-2.86223,-3.09582,-1.84317,1.7732,-0.0332241],
[-0.812984,0.556779,0.0258276,-2.67347,1.47472,2.8966,1.09251],
[1.27589,1.95198,0.757287,-1.25869,-1.10887,2.91714,0.362636],
[2.35275,-1.35469,-0.870603,-2.05875,-2.402,-0.117067,-0.237904],
[2.62631,-1.7628,-1.48536,-1.62642,-1.76757,1.47608,1.87821]]

,dtype=float)

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
