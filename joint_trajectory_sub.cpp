// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <vector>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <signal.h>
#include "spline.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

void ctrlchandler(int){exit(EXIT_SUCCESS);}
void killhandler(int){exit(EXIT_SUCCESS);}
int trig_command = 0;
std::vector<std::array<double,7>> q_list;
std::array<double, 7> q_goal = {{0, 0, 0, 0, 0, 0, 0}};
std::array<double, 7>  move_joint_values= {{0.000 ,-0.785, 0.000, -2.356, 0.000, 1.571 ,1.585}};
void print_q(std::array<double, 7> q ){
	std::cout<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<","<<q[4]<<","<<q[5]<<","<<q[6]<<std::endl;
}
void rosJointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
	q_list.clear();
	trig_command++;
	for(int j =0;j<msg->points.size();j++){
		for(int i = 0;i<7;i++){
			q_goal[i] = msg->points[j].positions[i];
		}
		q_list.push_back(q_goal);
	}
}

std::vector<std::array<double,7>> splineJointTrajectory(std::vector<std::array<double,7>> q_list,double Tf, double dt, int deriv_num) {


   int N = q_list.size();
   std::vector<double> Tlist;

   for(int j=0;j<N;j++){
	Tlist.push_back(double(Tf/(N-1)*j/1.0));
   }		
   std::cout<<"-----------Tlist-----------"<<std::endl;
   std::cout<<"-----------thetalist-----------"<<q_list.size()<<std::endl;
   std::vector<std::vector<double>> all_spline_thetalist;
   std::vector<std::vector<double>> all_spline_dthetalist;

   for(int j=0;j<7;j++){
	   std::vector<double> thetalist;
	   for(int i = 0;i<N;i++){
		   std::array<double,7> temp=q_list.at(i);
	  	   thetalist.push_back(temp[j]);
	   }
	   tk::spline s(Tlist,thetalist,tk::spline::cspline,false, tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
           std::vector<double> spline_thetalist;
           std::vector<double> spline_dthetalist;

	   for(double t=0+dt;t<=Tf;){
		spline_thetalist.push_back(s(t));
		spline_dthetalist.push_back(s.deriv(1,t));
		t = t+dt;
	   }
        all_spline_thetalist.push_back(spline_thetalist);
        all_spline_dthetalist.push_back(spline_dthetalist);
   }
   std::vector<std::array<double,7>>  spline_q_list;
   std::vector<std::array<double,7>>  spline_dq_list;

   for(int i=0;i<all_spline_thetalist.at(0).size();i++){
           std::array<double,7> temp;
	   for(int j=0;j<7;j++){
		 std::vector<double>temp_ = all_spline_thetalist.at(j);
		 temp[j]=temp_[i];
	   }
	   spline_q_list.push_back(temp);
   }

   for(int i=0;i<all_spline_dthetalist.at(0).size();i++){
           std::array<double,7> temp;
	   for(int j=0;j<7;j++){
		 std::vector<double>temp_ = all_spline_dthetalist.at(j);
		 temp[j]=temp_[i];
	   }
	   spline_dq_list.push_back(temp);
   }

   if (deriv_num==0)
	   
	   return spline_q_list;
   else if (deriv_num==1)
	   return spline_dq_list;

}


namespace {
class Controller {
 public:
  Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,  // NOLINT(readability-identifier-naming)
             const std::array<double, 7>& K_D)  // NOLINT(readability-identifier-naming)
      : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
  }
  inline franka::Torques step(const franka::RobotState& state) {
    updateDQFilter(state);
    std::array<double, 7> tau_J_d;  // NOLINT(readability-identifier-naming)
    for (size_t i = 0; i < 7; i++) {
      tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));
    }
    return tau_J_d;
  }
  void updateDQFilter(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }
  double getDQFiltered(size_t index) const {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }
 private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;
  const std::array<double, 7> K_P_;  // NOLINT(readability-identifier-naming)
  const std::array<double, 7> K_D_;  // NOLINT(readability-identifier-naming)
  std::array<double, 7> dq_d_;
  std::unique_ptr<double[]> dq_buffer_;
};
std::vector<double> generateTrajectory(double a_max) {
  // Generating a motion with smooth velocity and acceleration.
  // Squared sine is used for the acceleration/deceleration phase.
  std::vector<double> trajectory;
  constexpr double kTimeStep = 0.001;          // [s]
  constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
  constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
  // obtained during the speed up
  // and slow down [rad/s^2]
  double a = 0;  // [rad/s^2]
  double v = 0;  // [rad/s]
  double t = 0;  // [s]
  while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
    if (t <= kAccelerationTime) {
      a = pow(sin(t * M_PI / kAccelerationTime), 2) * a_max;
    } else if (t <= (kAccelerationTime + kConstantVelocityTime)) {
      a = 0;
    } else {
      const double deceleration_time =
          (kAccelerationTime + kConstantVelocityTime) - t;  // time spent in the deceleration phase
      a = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;
    }
    v += a * kTimeStep;
    t += kTimeStep;
    trajectory.push_back(v);
  }
  return trajectory;
}
}  // anonymous namespace
int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <simulation> <end_time>" << std::endl;
    return -1;
  }
  int is_sim=atoi(argv[2]);
  double end_time = atoi(argv[3]);

  if(is_sim==0){


	std::cout<<"This is Real Robot Program"<<std::endl;
	// Parameters
	const size_t joint_number{3};
	const size_t filter_size{5};
	// NOLINTNEXTLINE(readability-identifier-naming)
	const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
	// NOLINTNEXTLINE(readability-identifier-naming)
	const std::array<double, 7> K_D{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
	const double max_acceleration{1.0};
	Controller controller(filter_size, K_P, K_D);
	franka::Robot robot(argv[1]);
	setDefaultBehavior(robot);
	std::cerr << "Robot is Connected "  << std::endl;
	try{
		ros::init(argc,argv,"trajectory_subscriber");
	}
	catch(int e){
		ctrlchandler(1);

	}
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("joint_trajectory", 1,rosJointTrajectoryCallback); 
	ros::Publisher pub =  nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Rate r(10);
	std::cout<<"ROS JOINT TRAJECTORY SUBSCRIBER IS ON"<<std::endl;

	sensor_msgs::JointState jointState;
	jointState.name.push_back("panda_joint1");
	jointState.name.push_back("panda_joint2");
	jointState.name.push_back("panda_joint3");
	jointState.name.push_back("panda_joint4");
	jointState.name.push_back("panda_joint5");
	jointState.name.push_back("panda_joint6");
	jointState.name.push_back("panda_joint7");
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	while(ros::ok()){


		size_t count = 0;
		robot.read([&count](const franka::RobotState& robot_state) {
			move_joint_values=robot_state.q;
			return count++ < 2;
		});
		//print_q(move_joint_values);
		jointState.header.stamp = ros::Time::now();
		jointState.position[0] = move_joint_values[0];
		jointState.position[1] = move_joint_values[1];
		jointState.position[2] = move_joint_values[2];
		jointState.position[3] = move_joint_values[3];
		jointState.position[4] = move_joint_values[4];
		jointState.position[5] = move_joint_values[5];
		jointState.position[6] = move_joint_values[6];
		pub.publish(jointState);

		if(trig_command==1 && q_list.size()>0){
			try {

				// First move the robot to a suitable joint configuration
				std::array<double, 7> q_goal = q_list.at(0);
				MotionGenerator motion_generator(0.5, q_goal);
				std::cout << "WARNING: This example will move the robot! "
				      << "Please make sure to have the user stop button at hand!" << std::endl
				      << "Press Enter to continue..." << std::endl;   
				robot.control(motion_generator);
				std::cout << "Finished moving to initial joint configuration." << std::endl;
				// Set additional parameters always before the control loop, NEVER in the control loop!
				// Set collision behavior.
				//robot.setCollisionBehavior(
				//{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
				//{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
				//{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
				//{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
				
				
				    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

				size_t index = 0;
				//std::vector<double> trajectory = generateTrajectory(max_acceleration);

			    double dt = 0.001;
				if(q_list.size()<3){
					q_list.push_back(q_list.at(q_list.size()-1));
				}

		            std::vector<std::array<double,7>> trajectory = splineJointTrajectory(q_list,end_time,dt, 1);
			    robot.control([&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {return controller.step(robot_state);},
				[&](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
					  index += period.toMSec();
					  if (index >= trajectory.size()) {
					    index = trajectory.size() - 1;
					  }
					  franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
 				          
					  velocities.dq[0] = trajectory.at(index)[0];
					  velocities.dq[1] = trajectory.at(index)[1];
					  velocities.dq[2] = trajectory.at(index)[2];
					  velocities.dq[3] = trajectory.at(index)[3];
					  velocities.dq[4] = trajectory.at(index)[4];
					  velocities.dq[5] = trajectory.at(index)[5];
					  velocities.dq[6] = trajectory.at(index)[6];
					  print_q(velocities.dq);
					  if (index >= trajectory.size() - 1) {
	  					  velocities.dq[0] = 0.0;
						  velocities.dq[1] = 0.0;
						  velocities.dq[2] = 0.0;
						  velocities.dq[3] = 0.0;
						  velocities.dq[4] = 0.0;
						  velocities.dq[5] = 0.0;
						  velocities.dq[6] = 0.0;
					    return franka::MotionFinished(velocities);
					  }
					  return velocities;
				});


			  } catch (const franka::ControlException& e) {
			    std::cout << e.what() << std::endl;
			    
			    return -1;
			  } catch (const franka::Exception& e) {
			    std::cout << e.what() << std::endl;
			    return -1;
			  }
		trig_command = 0;
		q_list.clear();
		}

	ros::spinOnce();
	r.sleep();
	}

 }//is sim

else{


	try{ros::init(argc,argv,"trajectory_subscriber");}
	catch(int e){ctrlchandler(1);}
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("joint_trajectory", 1,rosJointTrajectoryCallback); 
	ros::Publisher pub =  nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Rate r(10);
	std::cout<<"ROS JOINT TRAJECTORY SUBSCRIBER IS ON"<<std::endl;
	sensor_msgs::JointState jointState;
	jointState.name.push_back("panda_joint1");
	jointState.name.push_back("panda_joint2");
	jointState.name.push_back("panda_joint3");
	jointState.name.push_back("panda_joint4");
	jointState.name.push_back("panda_joint5");
	jointState.name.push_back("panda_joint6");
	jointState.name.push_back("panda_joint7");
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	jointState.position.push_back(0.0);
	while (ros::ok()){

		if(trig_command==1 && q_list.size()>0){
				std::cout<<"QLIST SIZE :"<<q_list.size()<<std::endl;
			if(q_list.size()<3){
				q_list.push_back(q_list.at(q_list.size()-1));
				}
			
			double dt = 0.001;
		        std::vector<std::array<double,7>> trajectory = splineJointTrajectory(q_list,end_time,dt, 1);
			for(int index = 0;index<trajectory.size()-1;index++){
			  franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};

			  velocities.dq[0] = trajectory.at(index)[0];
			  velocities.dq[1] = trajectory.at(index)[1];
			  velocities.dq[2] = trajectory.at(index)[2];
			  velocities.dq[3] = trajectory.at(index)[3];
			  velocities.dq[4] = trajectory.at(index)[4];
			  velocities.dq[5] = trajectory.at(index)[5];
			  velocities.dq[6] = trajectory.at(index)[6];
			  //std::cout<<"index "<<index<<"  :  ";
			  //print_q(velocities.dq);
			}
			trig_command = 0;
			q_list.clear();
		}
		jointState.header.stamp = ros::Time::now();
		jointState.position[0] = move_joint_values[0];
		jointState.position[1] = move_joint_values[1];
		jointState.position[2] = move_joint_values[2];
		jointState.position[3] = move_joint_values[3];
		jointState.position[4] = move_joint_values[4];
		jointState.position[5] = move_joint_values[5];
		jointState.position[6] = move_joint_values[6];
		pub.publish(jointState);

		
		ros::spinOnce();
	}

}

   


}

