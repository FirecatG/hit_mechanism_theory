#include <cstdio>
#include <vector>
#include <cmath>
#include "four_bar_mechanism.h"
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

    return 0;
}
