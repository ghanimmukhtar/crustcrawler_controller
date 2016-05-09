#ifndef __SIMU_HPP__
#define __SIMU_HPP__

#include <robdyn/renderer/osg_visitor.hh>

#include <arm_controller.hpp>
#include "arm_crustcrawler.hpp"
#include "robotArm.hpp"
#include "parameters_simulation.hpp"
#include <robdyn/ode/box.hh>
#include <map>

class Simu {
	
public:
	
    Simu(){}
    Simu(Data d);
    Simu(Data d, float cube_x, float cube_y, float rotationCoeff);
	~Simu();
	
    void init(Data d, float cube_x = Params_Simu::cube::dist_arm_x, float cube_y = Params_Simu::cube::dist_arm_y, float rotationCoeff = Params_Simu::cube::rotationCoeff);

	/**
	 * @brief launch the simulation with the controller ac
	 * @param controller (few waypoints)
	 */
	void launch(Arm_controller &ac);
	
	/**
	 * @brief wait for the stabilization of the arm, used after a move
	 * @param intensity of the stabilization
	 * @return true if the arm stabilizes itself
	 */
	bool stabilize(int intensity);
	
	/**
	 * @brief wait for the stabilization of the cube, used after the whole move
	 * @param intensity of the stabilization
	 * @return true if the cube stabilizes itself
	 */
	bool stabilizeCube(int intensity);
	
	/**
	 * @brief return the current position of all motors of the arm
	 * @return position of the arm (waypoint format)
	 */
	waypoint getCurrentPos();
	
	/**
	 * @brief begins the move from current position to waypoint target
	 * @param the controller
	 * @param the waypoint planed to reach
	 */
	void goToWaypoint(Arm_controller ac, waypoint target);
	
	/**
	 * @brief upload the simulation
	 */
	void next_step();
	
	
	// Getters / Setters
	float getStep() { return _step; }
	void setStep(float step) { _step = step; }
	float getSpeed() { return _speed; }
	void setSpeed(float speed) { _speed = speed; }
	int getTimeOfStab() { return _timeOfStab; }
	void setTimeOfStab(int timeOfStab) { _timeOfStab = timeOfStab; }
	
	bool getCrash() { return _crash; }
	void setCrash(bool crash) { _crash = crash; }
	float getTotalTorque() { return _total_torque; }
	void setTotalTorque(float total_torque) { _total_torque = total_torque; }
	Eigen::Vector3d& getMin() { return _min; }
	void setMin(Eigen::Vector3d min) { _min = min; }
	Eigen::Vector3d& getMax() { return _max; }
	void setMax(Eigen::Vector3d max) { _max = max; }
	Eigen::Vector3d getDiffCoords() { return _diffCoords; }
	void setDiffCoords(Eigen::Vector3d diffCoords) { _diffCoords = diffCoords; }
    void setPerformance(float performance) { _performance = performance; }
	float getPerformance() { return _performance; }
	void setEffectorAngle(float effectorAngle) { _effectorAngle = effectorAngle; }
	float getEffectorAngle() { return _effectorAngle; }
	void setWristAngle(float wristAngle) { _wristAngle = wristAngle; }
	float getWristAngle() { return _wristAngle; }
	
    const std::map<float, std::vector<float> >& get_arm_trajectory(){return _arm_trajectory;}

	boost::shared_ptr<ode::Environment_arm> getEnv() { return _env; }
	void setEnv(boost::shared_ptr<ode::Environment_arm> env) { _env = env; }
    boost::shared_ptr<robot::Arm> getArm() { return _arm; }
    void setArm(boost::shared_ptr<robot::Arm> arm) { _arm = arm; }
	
	
private:
	
	float _step;
	float _speed;
	int _timeOfStab;
	
	bool _crash;
	float _total_torque;
	Eigen::Vector3d _min, _max;
	Eigen::Vector3d _diffCoords;
	float _performance;
	float _effectorAngle;
	float _wristAngle;
    std::map<float,std::vector<float> > _arm_trajectory;

    bool cube_touched;

	boost::shared_ptr<ode::Environment_arm> _env;
    boost::shared_ptr<robot::Arm> _arm;
	
	renderer::OsgVisitor _visitor;
};

#endif
