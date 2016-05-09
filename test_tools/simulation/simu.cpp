#include "simu.hpp"
#include <iostream>


Simu::Simu(Data d) {
    init(d);
}

Simu::Simu(Data d, float cube_x, float cube_y, float rotationCoeff){
    init(d,cube_x,cube_y,rotationCoeff);
}

Simu::~Simu() {
	
}

void Simu::init(Data d, float cube_x, float cube_y, float rotationCoeff){

    cube_touched = false;

    // GLOBAL
    setStep(0.015); // Step in millisecondes
    setSpeed(1.4); // Speed of the robot (in radian per second)
    setTimeOfStab(500); // Time let to the arm to stabilize itself

    // USEFULL VARIABLES
    setCrash(false);
    setTotalTorque(0);
    setMin(Eigen::Vector3d(100, 100, 100));
    setMax(Eigen::Vector3d(0, 0, 0));
    setEffectorAngle(-1);
    setWristAngle(-1);

    // ENVIRONMENT AND ARM
    setEnv(boost::shared_ptr<ode::Environment_arm>(new ode::Environment_arm()));
    setArm(boost::shared_ptr<robot::Arm>(new robot::Arm(*getEnv(), Params_Simu::table::height)));

    // ENVIRONMENT INITIALIZATION
    getEnv()->set_gravity(0, 0, -9.81);
    getEnv()->add_table(*getEnv(),
                            Params_Simu::table::length, Params_Simu::table::width, Params_Simu::table::height,
                            Eigen::Vector3d(Params_Simu::table::dist_arm_x, Params_Simu::table::dist_arm_y, Params_Simu::table::height/2));
    //add cube
    d.setRotation_coeff(rotationCoeff);
    getEnv()->add_cube(*getEnv(),
                            Eigen::Vector3d(cube_x, cube_y, Params_Simu::table::height),
                            Params_Simu::cube::size,
                            d.getRealCubeMass(),
                            d.getRealCubeRotation());



    getArm()->accept(_visitor);
    getEnv()->accept_table(_visitor);
    getEnv()->accept_cube(_visitor);


}

void Simu::launch(Arm_controller &ac) {
	// Quit if no controller
    //assert(ac.getNbWaypoints() > 0);
	
	
	// Launch all waypoint with stabilization between them (stab time explains a lot the time of Map-Elite process)
	if(!stabilize(Params_Simu::global::intensityOfStab/2)) { return; }
	
	Eigen::Vector3d posCube(0, 0, 0);
	for(int i(-1) ; i < ac.getNbWaypoints()-1 ; i++) {
//		std::cout << "[simulation] move from wp" << i << " to wp" << i+1 << std::endl;
		goToWaypoint(ac, ac.getWaypoint(i+1)); // From i to i+1 (i = -1 for origin)
		if(!stabilize(Params_Simu::global::intensityOfStab)) { return; }
	}
	
	if(!stabilizeCube(Params_Simu::global::intensityOfStab)) { return; }
	
	
	// Recover all data needed for (1) knowing the concerned cell and (2) the appropriated performance
	// (1) - 5 dims
	if(getEffectorAngle() == -1) {
		setEffectorAngle(0.5);
		setWristAngle(0.5);
	}
	// (1) - 6 dims
	Eigen::Vector3d diff;
	for(int i(0) ; i < 3 ; i++)
		diff[i] = getMax()[i]-getMin()[i];
	setDiffCoords(diff);
	
}

bool Simu::stabilize(int intensity) {
	bool stabilized(false);
	int stab(0);
	
	for (size_t s(0) ; s < getTimeOfStab() && !stabilized ; ++s) {
		Eigen::Vector3d posArm(getArm()->pos());
		
		next_step();
		if ((getArm()->pos() - posArm).norm() < 1e-4)
			stab++;
		else
			stab = 0;
		
		if (stab > intensity)
			stabilized = true;
	}
	
	if(!stabilized) {
		setCrash(true);
		return false;
	}
	
	return true;
}

bool Simu::stabilizeCube(int intensity) {
	bool stabilized(false);
	int stab(0);
	
	for (size_t s(0) ; s < getTimeOfStab() && !stabilized ; ++s) {
		Eigen::Vector3d posCube(getEnv()->getCubePos());
		
		next_step();
		if ((getEnv()->getCubePos() - posCube).norm() < 1e-3)
			stab++;
		else
			stab = 0;
		
		if (stab > intensity)
			stabilized = true;
	}
	
	if(!stabilized) {
		setCrash(true);
		return false;
	}
	
	return true;
}

waypoint Simu::getCurrentPos() {
	waypoint current;
	std::vector<float> tmp;
	for(int i(0) ; i < getArm()->servos().size() ; i++) {
		tmp.push_back((getArm()->servos()[i]->get_angle(0)/M_PI)+0.5);
	}
	
	current.step = tmp;
	current.fingersValue = 0;
	
	return current;
}

void Simu::goToWaypoint(Arm_controller ac, waypoint target) {
	
	int nbOfFingers(2), nbOfServosByFingers(1);
	int nbOfServosForFingers(nbOfFingers*nbOfServosByFingers);
	bool fingersDone(false);
	
	waypoint current = getCurrentPos();
	float moveTime(ac.getMoveTime(current, target, getSpeed()));
    float start_time = 0;
    if(!_arm_trajectory.empty()){
        std::map<float, std::vector<float> >::iterator it = _arm_trajectory.end();
        it--;
        start_time = it->first;
    }
	Eigen::Vector3d initCubePos(getEnv()->getCubePos());
	
	// The motor that reached the longest distance is the reference : all the other motors has to do their move in the same time
	for (float t(0) ; t < moveTime ; t += getStep()) {
        for(int i(0) ; i < getArm()->servos().size()-nbOfServosForFingers ; i++) {
			float prevAngle(M_PI*(current.step[i]-0.5)), currAngle(M_PI*(target.step[i]-0.5));
			float newAngle(prevAngle + (currAngle-prevAngle)/moveTime*(t+getStep()));
            getArm()->servos()[i]->set_angle(0, newAngle);
		}
		
		// Fingers action (open / close) can be done at any moment of the move
		if(!fingersDone && (t/moveTime > target.fingersValue || t+getStep() >= moveTime)) {
			for(int i(0) ; i < nbOfFingers ; i++) {
				int posServo = getArm()->servos().size() - nbOfServosForFingers + 1;
				float currAngle(M_PI*(target.step[6+i]-0.5));
				getArm()->servos()[posServo + nbOfServosByFingers*i - 1]->set_angle(0, currAngle);
			}
			fingersDone = true;
		}

        std::vector<float> currentAngles;
        for(int i = 0; i < getArm()->servos().size();i++)
            currentAngles.push_back(getArm()->servos()[i]->get_angle(0));


		next_step();
		
		
		// Check if the min / max position of the cube has changed and if the cube has moved
		Eigen::Vector3d pos(getEnv()->getCubePos());
		
		for(int i(0) ; i < 3 ; i++) {
			if(pos[i] < getMin()[i]) { getMin()[i] = pos[i]; }
			if(pos[i] > getMax()[i]) { getMax()[i] = pos[i]; }
		}
		
        if(getEffectorAngle() == -1 && (fabs(Params_Simu::cube::dist_arm_x-pos[0]) > 1e-3 ||
                                        fabs(Params_Simu::cube::dist_arm_y-pos[1]) > 1e-3 ||
                                        fabs(Params_Simu::cube::dist_arm_z-pos[2]) > 1e-3)) {
			float sum = 0;
			for(int i(1) ; i < getArm()->servos().size()-nbOfServosForFingers-1 ; i++) {
				sum += getArm()->servos()[i]->get_angle(0)/float(M_PI);
			}
			sum = round((sum*(-1))*1e3)/float(1e3) - 0.5;
			if(sum < 0) sum = 0;
			else if(sum > 1) sum = 1;
			setEffectorAngle(sum);
			
			float wrist(getArm()->servos()[getArm()->servos().size()-nbOfServosForFingers-1]->get_angle(0)/float(M_PI) + 0.5);
			wrist = (wrist - 0.1) / 0.8f;
			wrist = round(wrist*1e3)/float(1e3);
			if(wrist < 0) wrist = 0;
			else if(wrist > 1) wrist = 1;
			setWristAngle(wrist);
		}

        if(/*!cube_touched &&*/ (fabs(initCubePos[0]-pos[0]) > 1e-3 ||
            fabs(initCubePos[1]-pos[1]) > 1e-3 ||
            fabs(initCubePos[2]-pos[2]) > 1e-3)){
            currentAngles.push_back(1);
            cube_touched = true;
        }
        else currentAngles.push_back(0);

        _arm_trajectory.insert(std::make_pair/*<float, std::vector<float> >*/(t + start_time,currentAngles));

	}

	// Update the sum of torques
	for(int i(0) ; i < getArm()->servos().size() ; i++)
		setTotalTorque(getTotalTorque()+getArm()->servos()[i]->get_torque());
}

void Simu::next_step() {
	getArm()->next_step(getStep());
    getEnv()->next_step(getStep());
	
	_visitor.update();
	usleep(1e4);
}



