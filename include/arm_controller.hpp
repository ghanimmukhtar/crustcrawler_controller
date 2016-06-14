#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <vector>
#include <Eigen/Geometry>


struct _waypoint {
	std::vector<double> step;
	double fingersValue; 
};
typedef _waypoint waypoint;


class Arm_controller {
	
public:
	
	Arm_controller();
	~Arm_controller();
	
	/**
	 * @brief prints the controller as used by the simulation (simulation format (8 double + 1 fingers value))
	 */
	void showControllers();
	
	/**
	 * @brief prints the controller as given by the user or by Map-Elite (user format (7 double))
	 */
	void showRealController();
	
	/**
	 * @brief add a waypoint to the controller
	 * @param the waypoint to add (user format (7 double))
	 */
	void addWaypoint(std::vector<double> step);
	
	/**
	 * @brief get the time needed to reach the waypoint b from the waypoint a
	 * @param the current position
	 * @param the goal (position to reach)
	 * @param the speed of the robot
	 * @return time needed
	 */
	double getMoveTime(waypoint a, waypoint b, double speed);
	
	/**
	 * @brief find the mean of the variances of all waypoints of the arm
	 * @return the variance
	 */
	double findVariance();
	
	/**
	 * @brief adapt the controller to the real arm
	 */
	void adaptToReality();
	
	
	// Getters / Setters
	void setNbWaypoints(int nbWaypoints) { _nbWaypoints = nbWaypoints; }
	int getNbWaypoints() { return _nbWaypoints; }
	void setOrigin(std::vector<double> origin) { _origin.step = origin; _origin.fingersValue = 0; }
	void setOrigin(double value) { _origin.step = _getVector(value); _origin.fingersValue = 0; }
	waypoint getOrigin() { return _origin; }
	void setTrajectory(std::vector< waypoint > trajectory) { _trajectory = trajectory;}
	std::vector< waypoint >& getTrajectory() { return _trajectory; }
	void setWaypoint(waypoint wp, int i) { _trajectory[i] = wp; }
	waypoint getWaypoint(int i) { if(i == -1) { return _origin; }
									else { return _trajectory[i]; } }
	
	
private:
	
	int _sign(double a);
	std::vector<double> _getVector(double val);
	double _variance(std::vector<double> v);
	double _mean(std::vector<double> v);
	
	int _nbWaypoints;
	waypoint _origin;
	std::vector< waypoint > _trajectory;
};
#endif
