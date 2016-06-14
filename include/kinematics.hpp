#ifndef __KINEMATICS_HPP__
#define __KINEMATICS_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include "robotArm.hpp"
#include "real.hpp"
#include "robot_params.hpp"

/**
 * @brief The ArmTrajectory class
 * This class provide the tools to control the fake crustcrawler in morse simulator (https://www.openrobots.org/morse/doc/stable/morse.html).
 */
class Kinematics {

private :
    //This function returns the transformation matrix between two consecutive frames whose geometric parameters are given by the vector kk
    Eigen::Matrix4d trmat_kk(const Eigen::VectorXd& kk) const;

    //This function returns the geometric parameters for actuators frames according to Khalil-Kleinfinger notation
    Eigen::MatrixXd kk_mat(const std::vector<double>& a) const;

    //This function returns all the intermediate transformation matrices between all joints, this will be used later to formulate the jacobian matrix
    std::vector<Eigen::Matrix4d> sub_trmat(const std::vector<double>& a) const;


    //variables definition
    std::vector<double> initial_pos;  //used to store the starting cartesian position of the arm's end effector, from which it will start moving towards desired position
    double rtdot;  //used to store, at each iteration, the current value of the first derivative of a fifth degree polynomial interpolation
    double duration; //defines the duration, in seconds, for executing the trajectory from the initial_pos to the target position
    const double max_speed = 1.;
    const double max_load = 0.3;
public :

    Kinematics()
    {
        duration = 6;
    }

    /**
     * @brief initializing necessary variables to guide the arm to a desired Cartesian position.
     * @param desired position, x, y and z coordinates
     * @return nothing but initialization of appropriate variables
     */
    void init_motion(std::vector<double> desired_position);

    /**
     * @brief compute jacobian matrix for the current arm configurations (current joints angle).
     * @param joint values in radian, they have to be with respect to the starting configuration of the arm for which the geometric Parameters has been defined
     * @return the jacobian matrix
     */
    Eigen::MatrixXd jacobian(const std::vector<double>& a) const;

    /**
     * @brief pseudo_inverse
     * @param mat
     * @param invMat
     */
    void pseudo_inverse(const Eigen::MatrixXd& mat, Eigen::MatrixXd& invMat);

    /**
     * @brief compute the forward model.
     * @param joint values in radian
     * @return approximate effector position in meter
     */
    std::vector<double> forward_model(const std::vector<double>& a) const;

    /**
     * @brief guide the arm to a desired joints angles q(q1,q2,q3,q4,q5,q6) in velocity mode for each actuator.
     * @param desired joints positions (waypoint), a vector (q)
     * @return nothing but it guides the arm to the desired joints angles in velocity mode
     */
    void goto_desired_joints_angles_velocity_mode(std::vector<double> desired_joints_angles);

    /**
     * @brief guide the arm to a desired joints angles q(q1,q2,q3,q4,q5,q6) in position mode for each actuator.
     * @param desired joints positions (waypoint), a vector (q)
     * @return nothing but it guides the arm to the desired joints angles in position mode
     */
    void goto_desired_joints_angles_position_mode(std::vector<double> desired_joints_angles);

    /**
     * @brief This method performs the control of the robot in the 3D operational space, based on inversed jacobian. (The orientation of the gripper is not considered yet.)
     * @param 3 dimensional vector: target position
     */
    void control_inverse_initialize(std::vector<double> target, std::vector<double> initial_joint_pos);

    std::vector<double> control_inverse(std::vector<double> actual_joint_pos, double duration, double current_time);

    /**
     * @brief use invers kinematic method to guide the arm to a desired Cartesian position.
     * @param desired position, x, y and z coordinates
     * @return nothing but guide the arm to the desired position
     */
    void goto_desired_position(std::vector<double> desired_position);

    /**
     * @brief positioning the gripper to a predefined pose.
     * @param no parameters needed
     * @return nothing but guide the gripper to desired pose
     */
    void position_gripper();

    /**
     * @brief performs a primitive motion.
     * @param no parameters needed,
     * @return nothing but guide the arm into the desired direction
     */
    void primitive_motion();

    /**
     * @brief performs a retraction motion to releave the arm when it is stressed.
     * @param no parameters needed,
     * @return nothing but guide the arm so that it is releaved
     */
    void releave();

    //getter for the duration
    double get_duration()
    { return duration; }

    void set_duration(double d)
    { duration = d; }
};

#endif
