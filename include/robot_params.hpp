#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <string>

/**
 * @brief Params struct
 * In this struct is listed all parameters about the crustcrawler robot.
 * It's use for the forward model, inverse model and simulation.
 */
struct Params {
    // Plinth
    static constexpr double  body_width = 0.2;
    static constexpr double  body_height = 0.055/*0.043*/;
    static constexpr double  body_length = 0.2;
    static constexpr double  body_mass = 20;

    // Pivot
    static constexpr double  p1_width = 0.13;
    static constexpr double  p1_height = 0.057/*0.082*/;
    static constexpr double  p1_length = 0.13;
    static constexpr double  p1_mass = 0.180;

    // Arm
    static constexpr double  p2_width = 0.051;
    static constexpr double  p2_height = 0.224;
    static constexpr double  p2_length = 0.04;
    static constexpr double  p2_mass = 0.110;

    static constexpr double  p3_width = 0.051;
    static constexpr double  p3_height = 0.157;
    static constexpr double  p3_length = 0.04;
    static constexpr double  p3_mass = 0.100;

    static constexpr double  p4_width = 0.051;
    static constexpr double  p4_height = 0.078;
    static constexpr double  p4_length = 0.04;
    static constexpr double  p4_mass = 0.100;

    static constexpr double  p5_width = 0.049;
    static constexpr double  p5_height = 0.074/*0.065*/;
    static constexpr double  p5_length = 0.05;
    static constexpr double  p5_mass = 0.078;

    // Second pivot (wrist)
    static constexpr double  p6_width = 0.061;
    static constexpr double  p6_height = 0.07;
    static constexpr double  p6_length = 0.034;
    static constexpr double  p6_mass = 0.078;


    // Gripper
    static constexpr double  gripper_height = 0.16;

    // Fingers
    static constexpr double  finger_width = 0.022;
    static constexpr double  finger_height = 0.095;
    static constexpr double  finger_length = 0.034;
    static constexpr double  finger_mass = 0.085;
    static constexpr double  finger_inter_dist = 0.05;

    //ik parameters
    static constexpr double gain = 0.05;
    static constexpr double max_iteration = 2500;
    static constexpr double th_error = 0.01;

    //Dynamixel parameters
//    static const char* dynamixel_serial = "/dev/ttyUSB0";
    static constexpr double MX_step_per_turn = 4096.0;
    static constexpr double AX_step_per_turn = 1024.0;
    static constexpr float read_duration = 0.02f;
};

#endif

