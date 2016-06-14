#include <iostream>
#include <cmath>
#include "../include/arm_controller.hpp"
#include "../include/kinematics.hpp"
#include "robotArm.hpp"

int main(int argc, char **argv)
{
    Kinematics km;
    std::vector<float> start_pose = {0.05,-0.15,0.2};
    std::vector<float> pose = {atof(argv[1]),atof(argv[2]),atof(argv[3])};
    std::vector<float> home_position = {0.05,-0.15,0.2};

    //use the method goto_desired_position(std::vector<float> target_position) to guide the end effector to the desired position
    km.goto_desired_position(start_pose);
    km.goto_desired_position(pose);
    km.goto_desired_position(home_position);
    km.position_gripper();
    return 0;
    /*
    //These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
    unsigned char tmp1[] = {1, 2, 3, 4, 5, 6, 7};
    std::vector<unsigned char> reduced_actuator_id (tmp1, tmp1 + 7);
    float tmp[] = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> initial_joint_values (tmp, tmp + 8);
    float prim_distance = -0.05;
    //These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper

    //instantiate a variable of kinematic class

    std::vector<float> joints_speeds(reduced_actuator_id.size(),0.0);
    std::vector<float> joints_velocity,last_angles,my_last_angles,target_position(3),home_position,current_position;

//    duration = d;
    Real robot;
    //set motors speed to zero so that they start from static situ
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];

    //initialize the inverse kinematic to get the distance to be covered
    km.control_inverse_initialize(pose,last_angles);
    bool stressed = false;
    //avariable to read joints loads and show them
    std::vector <float> joints_loads;
    //perform the trajectory in the desired time:
    float duration = km.public_duration;
    //first initialize a timer
    boost::timer initial_time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(initial_time_here.elapsed() < duration)
    {
        joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++){
            std::cout << joints_loads[i] << std::endl;
            if (joints_loads[i] > 0.3){
                std::cout << "i am out of here ******************************" << std::endl;
                stressed = true;
                break;
            }
        }
        if (stressed)
            break;
        std::cout << "*****************************************" << std::endl;
        //at each iteration get the current joint angles, as described before
        my_last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < my_last_angles.size(); i++)
            last_angles[i] = M_PI*my_last_angles[i] - initial_joint_values[i];
        joints_velocity = km.control_inverse(last_angles,duration,initial_time_here.elapsed());
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
            //std::cout << joints_loads[i] << std::endl;
        //std::cout << "*****************************************" << std::endl;
    }
    std::cout << "i am here ******************************" << std::endl;
    //set motors speed to zero so that they start from static situ
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    if (stressed)
    {
        std::cout << "i am stressed ******************************" << std::endl;
        //do the retract motion and then up
        last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < last_angles.size(); i++)
            last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];
        current_position = km.forward_model(last_angles);

        //get the diagonal line from the origin to the end effector
        double angle = atan2(current_position[1],current_position[0]);

        //set the target position which will have the same z coordinate but x and y will change according to the direction defined above
        target_position[0] = current_position[0] + prim_distance*cos(angle);
        target_position[1] = current_position[1] + prim_distance*sin(angle);
        target_position[2] = current_position[2];

        //perform the motion using the method goto_desired_position
        km.goto_desired_position(target_position);
        km.goto_desired_position(home_position);
        stressed = false;
    }
    //the end effector should be now at the desired position, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
    */
}
