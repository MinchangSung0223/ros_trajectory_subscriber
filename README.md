
# ros_trajectory_subscriber
## Ros jointTrajectory Subscriber
```bash
cd /usr/include; ln -s eigen3/Eigen Eigen;
  cmake .
  make -j16
  
  ./joint_trajectory_sub 172.16.0.2 0 10
  
```
## ROS jointTrajectory Publisher

입력된 각도로 부터 끝단만 45도 이동하는 30개의 trajectory 생성
```bash

  ./joint_trajectory_pub 0.000 -0.785 0.000 -2.356 0.000 1.571 1.585
   
```
python publisher
```bash
 ./joint_publisher.py  0.000 -0.785 0.000 -2.356 0.000 1.571 1.585
```
