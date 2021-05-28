#!/usr/bin/python
import sys
import rospy, math, time
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



joint_trajectory_list = np.array([[2.8973,-1.63979,-1.56705,-1.17703,-1.63596,1.5938,1.81668],
[1.60893,0.583161,0.814044,0.835969,-1.45621,-0.824362,0.763496],
[-1.10294,-1.23448,2.46961,-0.379278,1.25812,-2.87126,1.68482],
[1.05418,0.716588,0.0136085,1.35809,-0.293144,-1.25359,-0.46885],
[-1.00059,0.834985,2.52532,3.11125,-0.19426,0.193987,-2.24864],
[-0.725241,1.22061,-0.401195,2.86259,0.207394,-0.724093,-0.964207],
[1.12382,2.60582,-1.39422,-0.0247641,3.0113,0.281219,-1.59534],
[3.00591,1.18449,-1.5888,-0.219568,-0.216936,-1.2262,-2.35523],
[2.31172,1.32589,-1.69046,-1.74543,-2.13463,-0.604735,0.376721],
[1.87258,-1.7603,-1.41833,-1.20269,-1.7995,1.50306,0.696329]]
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
