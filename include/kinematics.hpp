#ifndef __KINEMATICS_HPP__
#define __KINEMATICS_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/timer.hpp>
#include "robotArm.hpp"
#include "real.hpp"
#include "robot_params.hpp"

/**
 * @brief The ArmTrajectory class
 * This class provide the tools to control the fake crustcrawler in morse simulator (https://www.openrobots.org/morse/doc/stable/morse.html).
 */
class Kinematics{

private :
    //This function returns the transformation matrix between two consecutive frames whose geometric parameters are given by the vector kk
    Eigen::Matrix4d trmat_kk(const Eigen::VectorXd& kk)const;
    //This function returns the geometric parameters for actuators frames according to Khalil-Kleinfinger notation
    Eigen::MatrixXd kk_mat(const std::vector<float>& a) const;
    //This function returns all the intermediate transformation matrices between all joints, this will be used later to formulate the jacobian matrix
    std::vector<Eigen::Matrix4d> sub_trmat(const std::vector<float>& a) const;


    //variables definition
    std::vector<float> initial_pos;  //used to store the starting cartesian position of the arm's end effector, from which it will start moving towards desired position
    float rtdot;  //used to store, at each iteration, the current value of the first derivative of a fifth degree polynomial interpolation
    float duration; //defines the duration, in seconds, for executing the trajectory from the initial_pos to the target position

public :

    Kinematics(){
        duration = 6;
    }

    /**
     * @brief initializing necessary variables to guide the arm to a desired Cartesian position.
     * @param desired position, x, y and z coordinates
     * @return nothing but initialization of appropriate variables
     */
    void init_motion(std::vector<float> desired_position);

    /**
     * @brief compute jacobian matrix for the current arm configurations (current joints angle).
     * @param joint values in radian, they have to be with respect to the starting configuration of the arm for which the geometric Parameters has been defined
     * @return the jacobian matrix
     */
    Eigen::MatrixXd jacobian(const std::vector<float>& a) const;

    /**
     * @brief compute the forward model.
     * @param joint values in radian
     * @return approximate effector position in meter
     */
    std::vector<float> forward_model(const std::vector<float>& a) const;

    /**
     * @brief guide the arm to a desired joints angles q(q1,q2,q3,q4,q5,q6) in velocity mode for each actuator.
     * @param desired joints positions (waypoint), a vector (q)
     * @return nothing but it guides the arm to the desired joints angles in velocity mode
     */
    void goto_desired_joints_angles_velocity_mode(std::vector<float> desired_joints_angles);

    /**
     * @brief guide the arm to a desired joints angles q(q1,q2,q3,q4,q5,q6) in position mode for each actuator.
     * @param desired joints positions (waypoint), a vector (q)
     * @return nothing but it guides the arm to the desired joints angles in position mode
     */
    void goto_desired_joints_angles_position_mode(std::vector<float> desired_joints_angles);

    /**
     * @brief This method performs the control of the robot in the 3D operational space, based on inversed jacobian. (The orientation of the gripper is not considered yet.)
     * @param 3 dimensional vector: target position
     */
    void control_inverse_initialize (std::vector<float> target, std::vector<float> initial_joint_pos);
    std::vector<float> control_inverse(std::vector<float> actual_joint_pos, float duration, double current_time);

    /**
     * @brief use invers kinematic method to guide the arm to a desired Cartesian position.
     * @param desired position, x, y and z coordinates
     * @return nothing but guide the arm to the desired position
     */
    void goto_desired_position(std::vector<float> desired_position);

    //getter for the duration
    float get_duration(){return duration;}
    void set_duration(float d){duration = d;}
};

#endif
