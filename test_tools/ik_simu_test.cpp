#include <iostream>
#include <cmath>

#include "simulation/simu.hpp"
#include "arm_controller.hpp"
#include "simulation/parameters_simulation.hpp"
#include "kinematics.hpp"


int main(int argc, char** argv){

    if(argc != 4){
        std::cout  << "usage : 3 floats for 3d postion" << std::endl;
        exit(1);
    }

    dInitODE();

    Data d;

    float tmp[] = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> initial_joint_values (tmp, tmp + 8);


    //These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
    unsigned char actuators_id[] = {1, 2, 3, 4, 5, 6, 7};
    std::vector<unsigned char> reduced_actuator_id (actuators_id, actuators_id + 7);

    std::vector<float> goal_pose(3);
    goal_pose[0] = atof(argv[1]);
    goal_pose[1] = atof(argv[2]);
    goal_pose[2] = atof(argv[3]);

    std::cout << "Launch the Simulator" << std::endl;
    std::cout << "End effector goes to ("
              << argv[1] << "," << argv[2] << "," << argv[3]
              << ") using ik." << std::endl;

    Simu simu(d);

    std::cout << "go to start position" << std::endl;
    simu.getArm()->servos()[2]->set_angle(0,-.5*M_PI);
    simu.next_step();

    sleep(2);

    for(int i = 0; i < simu.getArm()->servos().size(); i++)
        simu.getArm()->servos()[i]->set_mode(ode::Servo::M_VEL);


    std::vector<float> last_angles(8),joints_velocity(7);


    Kinematics kine;

//    duration = d;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++)
        simu.getArm()->servos()[k]->set_vel(0,0.0);



    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.

    for(int i = 0; i < simu.getArm()->servos().size(); i++)
        last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];

    kine.control_inverse_initialize(goal_pose,last_angles);

    float duration = kine.get_duration();

    std::cout << duration << std::endl;

    boost::timer timer;


    while(timer.elapsed() < duration){

        for(int i = 0; i < simu.getArm()->servos().size(); i++)
            last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);

        for (size_t i = 0; i < last_angles.size(); i++)
            last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];
        joints_velocity = kine.control_inverse(last_angles,duration,timer.elapsed());

//        for(auto v : joints_velocity)
//            std::cout << v << " ";
//        std::cout << std::endl;

        for(int i = 0; i < joints_velocity.size(); i++)
            simu.getArm()->servos()[i]->set_vel(0,joints_velocity[i]);
        simu.next_step();
    }

    dCloseODE();

    return 0;
}
