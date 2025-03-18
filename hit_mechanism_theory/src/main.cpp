
#include "ros/rate.h"
#include "ros/ros.h"
#include <cstdio>
#include <vector>
#include <cmath>
#include "four_bar_mechanism.h"
#include "sensor_msgs/JointState.h"
#include <iostream>

using namespace std;



int main(int argc, char* argv[]) {
    // init ROS

    
    // init object
    FourBarMechanism four_bar( 10.0, 75.0, 120.0, 135.0, 170.0, 20.0, 15.0);

    vector<Result> results = four_bar.calculate(0.0, 360.0, 1.0);

    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        std::printf("angle:%d   Fx:%.2f   Fy:%.2f   vFx:%.2f   vFy:%.2f   aFx:%.2f   aFy:%.2f\n",
                   static_cast<int>(i),
                   r.x_F, r.y_F, 
                   r.v_Fx, r.v_Fy,
                   r.a_Fx, r.a_Fy);
    }


   
    // ROS部分（仿真附加）
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate_one_second(1);
    ros::Rate loop_rate_100(100);

    // publish joint states
    double theta = 0 , init_rad = 1.41;
    four_bar.calculate(init_rad * 180 / M_PI, init_rad * 180 / M_PI, 1.0);// 计算初始角度(urdf初始值导致)
    double initial_pose[3] = {init_rad, four_bar.phi_[1], four_bar.phi_[2]};

    for(int i=1;i<=5;i++){
        ROS_INFO("publish will start in %dsec...",6-i);
        loop_rate_one_second.sleep();
    }

    
    while (ros::ok()) {
        theta += 1;
        if(theta >= 360)
            theta = 0;
        double theta_rad = theta * M_PI / 180;
        auto result = four_bar.calculate(theta, theta, 1.0);

        double rad1 = theta_rad - initial_pose[0], rad2 = four_bar.phi_[1] - initial_pose[1], rad3 = four_bar.phi_[2] - initial_pose[2];
        if(rad1 > M_PI)
            rad1 -= 2 * M_PI;
        else if(rad1 < -M_PI)
            rad1 += 2 * M_PI;
        if(rad2 > M_PI)
            rad2 -= 2 * M_PI;
        else if(rad2 < -M_PI)
            rad2 += 2 * M_PI;
        if(rad3 > M_PI)
            rad3 -= 2 * M_PI;
        else if(rad3 < -M_PI)
            rad3 += 2 * M_PI;
        ROS_INFO("Publishing angle:%f,%f,%f\n", rad1, rad2, rad3);

        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"jointA", "jointB", "jointD"};
        vector<double> pose = {rad1, rad2, rad3};

        joint_state.position = {pose[0], pose[1], pose[2]};
        
        pub.publish(joint_state);
        ros::spinOnce();
        // loop_rate_one_second.sleep();
        loop_rate_100.sleep();
    }
    

    return 0;
}
