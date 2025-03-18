#include "ros/ros.h"
#include <vector>
#include <cmath>
#include "four_bar_mechanism.h"
#include "sensor_msgs/JointState.h"

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
    ros::Rate loop_rate(1);

    // publish joint states
    double theta = 0;
    while (ros::ok()) {
        theta += 30;
        if (theta >= 360)
            theta = 0;
        double theta_rad = theta * M_PI / 180;
        auto result = four_bar.calculate(theta, theta, 1.0);
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"jointA", "jointB", "jointD"};
        joint_state.position = {theta_rad , four_bar.phi_[1] , four_bar.phi_[2]};
        
        pub.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Published joint states of angle %f", theta);
    }
    

    return 0;
}
