#include <iostream>
#include "../include/arm_controller.hpp"
#include "../include/kinematics.hpp"

int main(int argc, char **argv)
{    
    float tmp[] = {0.6, 0.1, 0.3, 0.0, 0.0, -1.5708};
    std::vector<float> desired_pose (tmp, tmp + 6);


    Kinematics km;

    km.goto_desired_position(desired_pose);

    return 0;
}
