#include <iostream>
#include <cmath>
#include <math.h>
#include "kinematics.hpp"

Kinematics km;
//Real my_robot;


std::vector<double> my_initial_joint_values = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> my_start_joint_values = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0};
std::vector<unsigned char> my_reduced_actuator_id = {1, 2, 3, 4, 5, 6, 7};
std::vector<double> joints_values(7), current_position;

/*void locate_end_effector(){
joints_values = my_robot.getArm().get_joint_values(my_reduced_actuator_id);
for(int i=0;i<joints_values.size();i++)
    std::cout << "current joint value for joint: " << i << " is: " << M_PI * joints_values[i] << std::endl;
for (int i = 0; i < joints_values.size(); i++) {
    joints_values[i] = M_PI * joints_values[i] - my_initial_joint_values[i];
}
current_position = km.forward_model(joints_values);
std::cout << "*************************************************" << std::endl;
std::cout << "X is: " << current_position[0] << std::endl
                         << "Y is: " << current_position[1] << std::endl
                            << "Z is: " << current_position[2] << std::endl;

}*/

int main(int argc, char **argv)
{
    std::vector<double> pose = {atof(argv[1]),atof(argv[2]),atof(argv[3])};
    //km.goto_desired_position(pose);
    //km.goto_desired_position_with_all_orientations(pose, true); //motion without push primitive (default is false)
    for(int i = 0; i < 10; i++){
        km.goto_desired_position_with_one_orientation(pose, false);
        std::cout << "*********************** This is iteration no: " << i << " *********************" << std::endl;
    }
    //std::vector<double> starting_joint_values = {0., -1.5708, 1.5708, 0.0, -1.5708, 0.0, 0.0, 0.0};
    //km.goto_desired_joints_angles_position_mode(starting_joint_values);
    //locate_end_effector();
    return 0;
}
