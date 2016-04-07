#ifndef ROBOTARM_HPP
#define ROBOTARM_HPP

#include <stdlib.h>
#include <math.h>
#include <dynamixel/dynamixel.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "arm_controller.hpp"

#define SAMPLING_FREQUENCY 20
#define DYNAMIXELSERIAL "/dev/ttyUSB0"
//#define DYNAMIXELSERIAL "/dev/ttyACM0"


struct Parameters {
	//Dynamixel parameters
    static constexpr double MX_step_per_turn = 4096.0;
    static constexpr double AX_step_per_turn = 1024.0;
    static constexpr float read_duration = 0.02f;
};

class RobotArm {

public:
	
	typedef unsigned char byte_t;

	RobotArm();
	~RobotArm();
	
	
	/**
	 * @brief initiatisation of connexion with all dynamixel
	 */
	void init();

	/**
	 * @brief reset positions of dynamixel motors at zero
	 */
	void reset();

	/**
	 * @brief disable all dynamixel motors
	 */
	void relax();

	/**
	 * @brief enable all dynamixel motors;
	 */
	void enable();
	
	/**
	 * @brief get current joint values of all dynamixel motors
	 * @return joint values in fractions of PI
	 */
	std::vector<float> get_joint_values(std::vector<byte_t> actuators_ids = std::vector<byte_t>());

    /**
     * @brief get current joint speeds of all dynamixel motors
     * @return joint speeds in RPM
     */
    std::vector<float> get_joint_speeds(std::vector<byte_t> actuators_ids);


	/**
	 * @brief send joint values to dynamixel motors
	 * @param controller vector of joint values in fraction of PI
	 */
	bool set_joint_values(std::vector<float> controller, std::vector<byte_t> actuators_ids = std::vector<byte_t>());

    /**
     * @brief send joint speeds to dynamixel motors
     * @param controller vector of joint speeds in fraction of PI.Sec
     */
    bool set_joint_speeds(std::vector<float> controller, std::vector<byte_t> actuators_ids = std::vector<byte_t>());

    /**
     * @brief change dynamixel motors operating mode to wheel mode, allowing their control in speed
     * @param motors IDs
     */
    bool mode_speed(std::vector<byte_t> reduced_actuator_id);

    /**
     * @brief change dynamixel motors operating mode to joint mode, allowing their control in position
     * @param motors IDs
     */
    bool mode_position(std::vector<byte_t> reduced_actuator_id);

	/**
	 * @brief change the value of P coefficient in PID motor regulation
	 * @param new value
	 */
	void changePValue(int val, int servo_index = -1);

	/**
	 * @brief change the value of I coefficient in PID motor regulation
	 * @param new value
	 */
	void changeIValue(int val, int servo_index = -1);

	/**
	 * @brief change the value of D coefficient in PID motor regulation
	 * @param new value
	 */
	void changeDValue(int val, int servo_index = -1);

	/**
	 * @brief close the connexion with the usb controllers
	 */
	void close_usb_controllers();
	
	
	// Getters / Setters
	dynamixel::Usb2Dynamixel& getController() { return _controller; }
	void setController(dynamixel::Usb2Dynamixel controller) { _controller = controller; }
	dynamixel::Status& getStatus() { return _status; }
	void setStatus(dynamixel::Status status) { _status = status; }
	std::vector<byte_t>& getActuatorsIds() { return _actuators_ids; }
	void setActuatorsIds(std::vector<byte_t> actuators_ids) { _actuators_ids = actuators_ids; }
	
	float getSpeed() { return _speed; }
	void setSpeed(float speed) { _speed = speed; }
	bool getCrash() { return _crash; }
	void setCrash(bool crash) { _crash = crash; }
	std::vector<float>& getErrors() { return _errors; }
	void setErrors(std::vector<float> errors) { _errors = errors; }
	void setErrors(int val) { for(int i(0) ; i < 8 ; i++) { _errors.push_back(val); } }


protected:

	dynamixel::Usb2Dynamixel _controller;
	dynamixel::Status _status;
	std::vector<byte_t> _actuators_ids;
	
	float _speed;
	bool _crash;
	std::vector<float> _errors;

	/**
	 * @brief conversion from fraction of PI to step per turn for MX dynamixel motor
	 * @param a
	 * @return
	 */
    int rad_to_stepperturn_MX(float a) { return Parameters::MX_step_per_turn/2*(1+a); }

	/**
	 * @brief conversion from fraction of PI to step per turn for AX dynamixel motor
	 * @param a
	 * @return
	 */
    int rad_to_stepperturn_AX(float a) { return Parameters::AX_step_per_turn/2*(1+a); }

    /**
     * @brief conversion from revolution per second (RPS) to revolution per minute (RPM) dynamixel motor speed
     * @param a
     * @return
     */
    int rps_to_rpm(float a) { return fabs(a)*60; }

    /**
     * @brief conversion from (RPM) dynamixel motor speed to (RPS)
     * @param a
     * @return
     */
    float rpm_to_rps(int a) { return a/60.0; }

	/**
	 * @brief conversion from MX step per turn to fraction of PI
	 * @param a
	 * @return
	 */
	float MX_stepperturn_to_rad(int a) { return 2*((float)a)/Parameters::MX_step_per_turn-1; }

	/**
	 * @brief conversion from AX step per turn to fraction of PI
	 * @param a
	 * @return
	 */
	float AX_stepperturn_to_rad(int a) { return 2*((float)a)/Parameters::AX_step_per_turn-1; }
};

#endif
