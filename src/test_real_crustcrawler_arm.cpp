#include <iostream>
#include <cmath>
#include "../include/arm_controller.hpp"
#include "../include/kinematics.hpp"

int main(int argc, char **argv)
{
    //give the desired position you want the end effector to go to, you only need to give the three Cartesian coordinates (i.e. x, y and z)
//    float tmp[] = {0.6, 0.0, 0.3};
//    std::vector<float> desired_pose (tmp, tmp + 3);
    std::vector<float> pose = {static_cast<float>(atof(argv[1])), static_cast<float>(atof(argv[2])),
                               static_cast<float>(atof(argv[3]))};
    //instantiate a variable of kinematic class
    Kinematics km;
    //use the method goto_desired_position(std::vector<float> target_position) to guide the end effector to the desired position
    km.goto_desired_position(pose);

//    //make the robot alternate between two positions (pos_1 = {0.6, 0.0, 0.3} & pos_2 = {0.42, 0.42 0.3})
//    for (int i=0;i < 4;i++){
//        if (i%2 == 0){
//            desired_pose[0] = 0.6*cos(0.78540);
//            desired_pose[1] = 0.6*sin(0.78540);
//        }
//        else{
//            desired_pose[0] = 0.6*cos(0);
//            desired_pose[1] = 0.6*sin(0);
//        }
//        desired_pose[2] = 0.3;
//        std::cout << "x coordinat for try " << i << " is: " << desired_pose[0] << std::endl;
//        std::cout << "y coordinat for try " << i << " is: " << desired_pose[1] << std::endl;
//        std::cout << " ********************************************** " << std::endl;
//        km.goto_desired_position(desired_pose);
//    }
    return 0;
}
