#ifndef __REAL_HPP__
#define __REAL_HPP__

#include "arm_controller.hpp"
#include "robotArm.hpp"

class Real {
	
public:
	typedef unsigned char byte_t;
	
	Real();
	~Real();
	
	/**
	 * @brief launch the simulation with the controller ac
	 * @param controller (few waypoints)
	 */
	void launch(Arm_controller ac);
	
	/**
	 * @brief returns to sleeping position
	 */
	void toSleepPosition();
	
	/**
	 * @brief returns to origin
	 */
	void toOrigin();

	/**
	 * @brief go to a given set of joint values (to a waypoint)
	 * @param the waypoint to reach
	 * @return false if it doesn't work (often when there is a problem with the dynamixels)
	 */
	bool goTo(waypoint target, std::vector<byte_t> actuators_ids = std::vector<byte_t>());

	/**
	 * @brief with the current start position and a goal position in joint space, this method compute a "step by step" trajectory
	 * @param goal position in joint space in fraction of PI
	 * @return a vector of vector of joint values in fraction of PI
	 */
	std::vector<std::vector<double> > getWaypoints(waypoint target, std::vector<byte_t> actuator_ids = std::vector<byte_t>());
	
	/**
	 * @brief wait for the stabilization of the arm
	 * @param the arm to stabilize
	 * @param the target to reached
	 * @param the intensity of the stabilization
	 */
	void stabilizeArm(RobotArm arm, std::vector<double> destination, int intensity);
	
	
	// Getters / Setters
	double getStep() { return _step; }
	void setStep(double step) { _step = step; }
	double getSpeed() { return _speed; }
	void setSpeed(double speed) { _speed = speed; }
	
	RobotArm& getArm() { return _arm; }
	void setArm(RobotArm arm) { _arm = arm; }
	Arm_controller& getAC() { return _ac; }
	void setAC(Arm_controller ac) { _ac = ac; }
	
	
private:
	
	double _step;
	double _speed;
	
	RobotArm _arm;
	Arm_controller _ac;
};

#endif
