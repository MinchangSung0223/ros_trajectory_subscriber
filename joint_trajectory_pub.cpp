// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <signal.h>
#include <stdio.h>
void ctrlchandler(int){exit(EXIT_SUCCESS);}
void killhandler(int){exit(EXIT_SUCCESS);}
int main(int argc, char** argv) {
	signal(SIGINT, ctrlchandler);
	signal(SIGTERM, killhandler);
	bool use_simulation = 0;
	
	try{
		ros::init(argc,argv,"trajectory_test");
	}
	catch(int e){
		ctrlchandler(1);
	
	}

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1000);
	ros::Rate r(30);
	
	std::cout<<"ROS JOINT TRAJECTORY PUBLISHER IS ON"<<std::endl;
	while (ros::ok()){
		trajectory_msgs::JointTrajectory jointTrajectory;

		jointTrajectory.joint_names.push_back("panda_joint1");
		jointTrajectory.joint_names.push_back("panda_joint2");
		jointTrajectory.joint_names.push_back("panda_joint3");
		jointTrajectory.joint_names.push_back("panda_joint4");
		jointTrajectory.joint_names.push_back("panda_joint5");
		jointTrajectory.joint_names.push_back("panda_joint6");
		jointTrajectory.joint_names.push_back("panda_joint7");
		jointTrajectory.header.stamp = ros::Time::now();
	
		trajectory_msgs::JointTrajectoryPoint points;
		for(int i = 1;i<31;i++){		
			points=trajectory_msgs::JointTrajectoryPoint();		
			points.positions.push_back(atof(argv[1]));
			points.positions.push_back(atof(argv[2]));
			points.positions.push_back(atof(argv[3]));
			points.positions.push_back(atof(argv[4]));
			points.positions.push_back(atof(argv[5]));
			points.positions.push_back(atof(argv[6]));
			points.positions.push_back(atof(argv[7])+3.141592/4/i);

			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);
			points.velocities.push_back(0.0);

			points.time_from_start = ros::Duration(0.01);

			jointTrajectory.points.push_back(points);
		}

			
		pub.publish(jointTrajectory);	
		ros::spinOnce();
	}
/*
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, 0, 0, 0, 0, 0, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 7> initial_position;
    double time = 0.0;
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4] , initial_position[5],
                                        initial_position[6] + delta_angle}};
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
*/
  return 0;
}
