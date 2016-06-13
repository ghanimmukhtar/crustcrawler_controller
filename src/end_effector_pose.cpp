#include <iostream>
#include <cmath>
#include "../include/arm_controller.hpp"
#include "../include/kinematics.hpp"

int main(int argc, char **argv)
{
    //give the desired position you want the end effector to go to, you only need to give the three Cartesian coordinates (i.e. x, y and z)
    //These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
    unsigned char tmp1[] = {1, 2, 3, 4, 5, 6, 7};
    std::vector<unsigned char> reduced_actuator_id (tmp1, tmp1 + 7);

    float tmp[] = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> initial_joint_values (tmp, tmp + 8);

    std::vector<float> pose;
    std::vector<float> joint_values(7);
    //instantiate a variable of kinematic class
    Kinematics km;
    Real arm;
    while(1){
        joint_values = arm.getArm().get_joint_values(reduced_actuator_id);

        for(int i = 0; i < joint_values.size(); i++)
            joint_values[i] = joint_values[i] * M_PI - initial_joint_values[i];

        //use the method goto_desired_position(std::vector<float> target_position) to guide the end effector to the desired position
        pose = km.forward_model(joint_values);

        std::cout << "end effector position : ";
        for(auto val : pose)
            std::cout << val << " ";
        std::cout << std::endl;

        sleep(0.5);
    }


    return 0;
}
