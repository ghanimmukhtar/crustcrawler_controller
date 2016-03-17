#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include <stdlib.h>
#include <math.h>
#include <dynamixel/dynamixel.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "robot_params.hpp"

#define DEFAULT_SPEED 150
#define SAMPLING_FREQUENCY 20
#define DYNAMIXELSERIAL "/dev/ttyACM0" //"/dev/ttyUSB0"
#define VMAX 0.5


namespace real{

/**
 * @brief The RobotArm class
 * This class provide all tools to control the crustcrawler (www.crustcrawler.com). This class use libdynamixel developped by J.B. Mouret (https://github.com/jbmouret/libdynamixel).
 */
class RobotArm
{
public:
    typedef unsigned char byte_t;

    RobotArm()
    {init();}

    ~RobotArm()
    {relax();}

    /**
     * @brief initiatisation of connexion with all dynamixel
     */
    void init();

    /**
     * @brief disable all dynamixel motors
     */
    void relax()
    {
        try{
            std::cout << "relax..." << std::endl;
            for (size_t i = 0; i < _actuators_ids.size(); ++i)
            {
                _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], false));
                _controller.recv(Params::read_duration, _status);
            }
            std::cout << "done" << std::endl;
        }catch(dynamixel::Error e){
            std::cerr << "dynamixel error : " << e.msg() << std::endl;
        }
    }

    /**
     * @brief enable all dynamixel motors;
     */
    void enable()
    {
        try{
            std::cout << "enable motor..." << std::endl;
            for (size_t i = 0; i < _actuators_ids.size(); ++i)
            {
                _controller.send(dynamixel::ax12::TorqueEnable(_actuators_ids[i], true));
                _controller.recv(Params::read_duration, _status);
            }
            std::cout << "done" << std::endl;
            usleep(1e5);
        }catch(dynamixel::Error e){
            std::cerr << "dynamixel error : " << e.msg() << std::endl;
        }
    }

    /**
     * @brief reset positions of dynamixel motors at zero
     */
    void reset();

    /**
     * @brief send to dynamixel motors the stable position for desactivation
     */
    void toSleepPosition();
    /**
     * @brief get current joint values of all dynamixel motors
     * @return joint values in fractions of PI
     */
    std::vector<float> get_joint_values(std::vector<byte_t> actuators_ids = std::vector<byte_t>());

    /**
     * @brief send instructions to open the gripper
     */
    void open_gripper();

    /**
     * @brief send instructions to close the gripper
     */
    void close_gripper();

    /**
     * @brief set a trajectory in joint space to a set position
     * @param ctrl vector of joint values in fraction of PI
     * @param if true then use inverse kinematic control else use direct control. set to false by default.
     */
    bool set_joint_trajectory(std::vector<float>& ctrl,  std::vector<byte_t> actuators_ids, bool inverse = false);
    bool set_joint_trajectory(std::vector<float>& ctrl,  bool inverse = false);

    /**
     * @brief send joint values to dynamixel motors
     * @param controller vector of joint values in fraction of PI
     */
    bool set_joint_values(std::vector<float> controller, std::vector<byte_t> actuators_ids = std::vector<byte_t>());

    /**
     * @brief number of dynamixel motors
     * @return number of actuators
     */
    size_t nb_actuators() const
    {
        return _actuators_ids.size();
    }

    /**
     * @brief close the connexion with the usb controllers
     */
    void close_usb_controllers()
    {

        _controller.close_serial();
    }

    /**
     * @brief with a given start position and a goal position in joint space, this method compute a "step by step" trajectory
     * @param start position in joint space in fraction of PI
     * @param goal position in joint space in fraction of PI
     * @return a vector of vector of joint values in fraction of PI
     */
    std::vector<std::vector<float> > getWayPoints(std::vector<float> goal, std::vector<byte_t> actuator_ids);

    /**
     * @brief final_pos (deprecated)
     * @return
     */
    const std::vector<float>& final_pos() const {return _final_pos;}

    /**
     * @brief isDead (deprecated)
     * @return
     */
    bool isDead()const {return _dead;}


    /**
     * @brief compute the forward model.
     * @param joint values in radian
     * @return approximate effector position in meter
     */
    std::vector<float> forward_model(const std::vector<float>& a) const;

    /**
     * @brief This method performs the control of the robot in the 3D operational space, based on inversed jacobian. (The orientation of the gripper is not considered yet.)
     * @param 3 dimensional vector: target position
     */
    std::vector<std::vector<float> > control_inverse(const std::vector<float>& target);

    /**
     * @brief compute the jacobian matrix
     * @param joint values in radian
     * @return jacobian matrix
     */
    Eigen::MatrixXd jacobian(const std::vector<float>& a) const;

protected:



    dynamixel::Usb2Dynamixel _controller;

    std::vector<float> _final_pos;
    dynamixel::Status _status;
    std::vector<byte_t> _actuators_ids;
    bool _dead;
    bool _is_close;

    std::vector<float> getDistances(std::vector<float> v1, std::vector<float> v2);
    float difference(float a, float b);

    //functions for inverse kinematic model.
    Eigen::Matrix4d trmat_dh(const Eigen::VectorXd& dh)const;
    Eigen::MatrixXd dh_mat(const std::vector<float>& a) const;
    std::vector<Eigen::Matrix4d> sub_trmat(const std::vector<float>& a) const;

    /**
     * @brief conversion from fraction of PI to step per turn for MX dynamixel motor
     * @param a
     * @return
     */
    int rad_to_stepperturn_MX(float a){return Params::MX_step_per_turn/2*(1+a);}

    /**
     * @brief conversion from fraction of PI to step per turn for AX dynamixel motor
     * @param a
     * @return
     */
    int rad_to_stepperturn_AX(float a){return Params::AX_step_per_turn/2*(1+a);}

    /**
     * @brief conversion from MX step per turn to fraction of PI
     * @param a
     * @return
     */
    float MX_stepperturn_to_rad(int a){return 2*((float)a)/Params::MX_step_per_turn-1;}

    /**
     * @brief conversion from AX step per turn to fraction of PI
     * @param a
     * @return
     */
    float AX_stepperturn_to_rad(int a){return 2*((float)a)/Params::AX_step_per_turn-1;}
};
}
#endif
