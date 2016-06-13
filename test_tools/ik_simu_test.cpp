#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include "simulation/simu.hpp"
#include "arm_controller.hpp"
#include "simulation/parameters_simulation.hpp"
#include "kinematics.hpp"


int main(int argc, char **argv)
{

    if (argc != 4) {
        std::cout << "usage : 3 floats for 3d postion" << std::endl;
        exit(1);
    }

    Arm_controller ac;
    dInitODE();

    Data d;

    double tmp[] = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> initial_joint_values(tmp, tmp + 8);


    //These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
    unsigned char actuators_id[] = {1, 2, 3, 4, 5, 6, 7};
    std::vector<unsigned char> reduced_actuator_id(actuators_id, actuators_id + 7);

    std::vector<double> goal_pose(3);
    goal_pose[0] = atof(argv[1]);
    goal_pose[1] = atof(argv[2]);
    goal_pose[2] = atof(argv[3]);


    std::cout << "Launch the Simulator" << std::endl;
    std::cout << "End effector goes to ("
    << argv[1] << "," << argv[2] << "," << argv[3]
    << ") using ik." << std::endl;

    Simu simu(d);

    std::cout << "go to start position" << std::endl;
    simu.getArm()->servos()[2]->set_angle(0, -0.5 * M_PI);
    //
    try {
        for (int i = 0; i < 100; i++) {
            simu.next_step();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (int e) {
        std::cout << "[simulation] exception met !" << std::endl;
    }

    sleep(2);


    for (int i = 0; i < simu.getArm()->servos().size(); i++) {
        simu.getArm()->servos()[i]->set_mode(ode::Servo::M_VEL);
    }


    std::vector<double> last_angles(8), joints_velocity(7);


    Kinematics kine;

//    duration = d;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++) {
        simu.getArm()->servos()[k]->set_vel(0, 0.0);
    }



    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.

    for (int i = 0; i < simu.getArm()->servos().size(); i++) {
        last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
    }
    for (int i = 0; i < last_angles.size(); i++) {
        last_angles[i] = last_angles[i] - initial_joint_values[i];

    }

    std::vector<double> start_pos = kine.forward_model(last_angles);

    std::cout << "start position is ";
    for (auto p : start_pos) {
        std::cout << p << " ";
    }
    std::cout << std::endl;


    kine.control_inverse_initialize(goal_pose, last_angles);

    double duration = kine.get_duration();

    std::cout << duration << std::endl;

    double timer = 0.0;

    while (timer < duration) {

        for (int i = 0; i < simu.getArm()->servos().size(); i++) {
            last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
        }

        for (size_t i = 0; i < last_angles.size(); i++) {
            last_angles[i] = last_angles[i] - initial_joint_values[i];
        }
        joints_velocity = kine.control_inverse(last_angles, duration, timer);

//        for(auto v : joints_velocity)
//            std::cout << v << " ";
//        std::cout << std::endl;

        for (int i = 0; i < joints_velocity.size(); i++) {
            simu.getArm()->servos()[i]->set_vel(0, joints_velocity[i]);
        }

        //update the simulation
        try {
            for (int i = 0; i < 10; i++) {
                simu.next_step();
                timer = timer + simu.getStep();
            }
        }
        catch (int e) {
            std::cout << "[simulation] exception met !" << std::endl;
        }

        //increment the timer by the simulation step time which actually defined as 0.015 milisec, in simu.cpp via the method setStep(double time)

    }


    for (int i = 0; i < simu.getArm()->servos().size(); i++) {
        last_angles[i] = simu.getArm()->servos()[i]->get_angle(0);
    }
    for (int i = 0; i < last_angles.size(); i++) {
        last_angles[i] = last_angles[i] - initial_joint_values[i];
    }
    std::vector<double> final_pos = kine.forward_model(last_angles);

    std::cout << "final position is ";
    for (auto p : final_pos) {
        std::cout << p << " ";
    }
    std::cout << std::endl;

    sleep(10);

    dCloseODE();

    return 0;
}
