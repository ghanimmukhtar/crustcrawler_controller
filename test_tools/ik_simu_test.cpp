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

    Arm_controller ac;
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
    simu.getArm()->servos()[2]->set_angle(0, -0.5*M_PI);
    //
    try {
        //simu.next_step();
        simu.launch(ac);
    }
    catch(int e) {
        std::cout << "[simulation] exception met !" << std::endl;
        }

    sleep(2);
    std::vector<float> last_angles(8),joints_velocity(7);
    Kinematics kine;
    //waypoint last_angles;
    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    for(int i = 0; i < simu.getArm()->servos().size(); i++)
        last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = last_angles[i] - initial_joint_values[i];
    std::vector<float> start_position = kine.forward_model(last_angles);
    Eigen::Vector3d real_start_pose = simu.getArm()->pos();
    std::cout << "real starting position is as follows : " << std::endl
              << "x = " << real_start_pose(0) << std::endl
              << "y = " << real_start_pose(1) << std::endl
              << "z = " << real_start_pose(2) << std::endl;

    std::cout << "starting position is as follows : " << std::endl
              << "x = " << start_position[0] << std::endl
              << "y = " << start_position[1] << std::endl
              << "z = " << start_position[2] << std::endl;
    kine.control_inverse_initialize(goal_pose,last_angles);
    float duration = kine.get_duration();
    for(int i = 0; i < simu.getArm()->servos().size(); i++)
        simu.getArm()->servos()[i]->set_mode(ode::Servo::M_VEL);
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++)
        simu.getArm()->servos()[k]->set_vel(0,0.0);

    float timer = 0.0;
    while(timer < duration){
        for(int i = 0; i < simu.getArm()->servos().size(); i++)
            last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
        for (size_t i = 0; i < last_angles.size(); i++)
            last_angles[i] = last_angles[i] - initial_joint_values[i];
        joints_velocity = kine.control_inverse(last_angles,duration,timer);
        for(int i = 0; i < joints_velocity.size(); i++)
            simu.getArm()->servos()[i]->set_vel(0,joints_velocity[i]);

        //update the simulation
        try {
            simu.next_step();
           }
        catch(int e) {
          std::cout << "[simulation] exception met !" << std::endl;
        }

        //increment the timer by the simulation step time which actually defined as 0.015 milisec, in simu.cpp via the method setStep(float time)
        timer = timer + simu.getStep();
    }

    for(int i = 0; i < simu.getArm()->servos().size(); i++)
        last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = last_angles[i] - initial_joint_values[i];

    std::vector<float> end_position = kine.forward_model(last_angles);

    Eigen::Vector3d real_end_pose = simu.getArm()->pos();
    std::cout << "real ending position is as follows : " << std::endl
              << "x = " << real_end_pose(0) << std::endl
              << "y = " << real_end_pose(1) << std::endl
              << "z = " << real_end_pose(2) << std::endl;

    std::cout << "ending position is as follows : " << std::endl
              << "x = " << end_position[0] << std::endl
              << "y = " << end_position[1] << std::endl
              << "z = " << end_position[2] << std::endl;

    dCloseODE();

    return 0;
}
