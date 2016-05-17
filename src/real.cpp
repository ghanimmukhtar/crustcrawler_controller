#include "../include/real.hpp"
#include <iostream>


Real::Real()
{
    setStep(0.015); // Step in millisecondes
    setSpeed(1.4); // Speed of the robot (in radian per second)
}

Real::~Real()
{

}

void Real::launch(Arm_controller ac)
{

    //assert(ac.getNbWaypoints() > 0);
    setAC(ac);

    getArm().reset();

    if (getArm().getCrash()) {
        return;
    }

    getArm().changePValue(16);
    getArm().changePValue(32, 2);
    getArm().changePValue(32, 3);
    getArm().changePValue(32, 4);
    getArm().changePValue(32, 6);

    getArm().changeIValue(0);
    getArm().changeIValue(25, 2);
    getArm().changeIValue(25, 3);

    getArm().changeDValue(0);
    getArm().changeDValue(50, 2);
    getArm().changeDValue(50, 3);
    getArm().changeDValue(30, 4);

    toSleepPosition();
    toOrigin();

    for (int i(0); i < getAC().getNbWaypoints(); i++) {
        std::cout << "[real_arm] go to wp" << i + 1 << std::endl;
        goTo(getAC().getWaypoint(i));
        stabilizeArm(getArm(), getAC().getWaypoint(i).step, 100);
    }

    setStep(0.01);
    toOrigin();
    setStep(0.005);
    toSleepPosition();

    getArm().relax();
    getArm().close_usb_controllers();
}

void Real::toSleepPosition()
{

    std::cout << "[real_arm] go to sleep position" << std::endl;

    waypoint sleep;
    std::vector<double> pos(8);
    pos[0] = 0.2;
    pos[1] = 0.1;
    pos[2] = 0.35;
    pos[3] = 0.45;
    pos[4] = 0.0;
    pos[5] = 0;
    pos[6] = .33;
    pos[7] = -.33;
    sleep.step = pos;
    sleep.fingersValue = 0;

    goTo(sleep);
    stabilizeArm(getArm(), sleep.step, 50);
}

void Real::toOrigin()
{

    std::cout << "[real_arm] go to origin" << std::endl;

    waypoint origin;
    std::vector<double> pos;
    for (int i(0); i < 6; i++) {
        pos.push_back(0);
    }
    pos.push_back(0.15);
    pos.push_back(-0.15);
    origin.step = pos;
    origin.fingersValue = 0;

    goTo(origin);
    stabilizeArm(getArm(), origin.step, 50);
}

bool Real::goTo(waypoint target, std::vector <byte_t> actuators_ids)
{
    if (actuators_ids.empty()) {
        actuators_ids = getArm().getActuatorsIds();
    }

    std::vector <std::vector<double>> waypoints;
    waypoints = getWaypoints(target, actuators_ids);

    for (int i(0); i < waypoints.size(); i++) {
        if (!getArm().set_joint_values(waypoints[i], actuators_ids)) {
            return false;
        }
    }

    return true;
}

std::vector <std::vector<double>> Real::getWaypoints(waypoint target, std::vector <byte_t> actuator_ids)
{

    waypoint start;
    start.step = getArm().get_joint_values(actuator_ids);
    start.fingersValue = 0;
    double moveTime(getAC().getMoveTime(start, target, getSpeed()));

    std::vector <std::vector<double>> waypoints;
    std::vector<double> final_pos(start.step.size());
    bool fingersDone(false);
    for (double t(0); t <= moveTime; t += getStep()) {
        waypoints.push_back(std::vector<double>());
        for (int i(0); i < start.step.size() - 2; i++) {
            final_pos[i] = start.step[i] + (target.step[i] - start.step[i]) / moveTime * (t/*+getStep()*/);
            waypoints.back().push_back(start.step[i] + (target.step[i] - start.step[i]) / moveTime * (t/*+getStep()*/));
        }
        if (!fingersDone and (t / moveTime > target.fingersValue or t + getStep() >= moveTime)) {
            for (int i(6); i <= 7; i++) {
                final_pos[i] = start.step[i] + (target.step[i] - start.step[i]);
                waypoints.back().push_back(final_pos[i]);
                start.step[i] = final_pos[i];
            }
            fingersDone = true;
        }
        else {
            for (int i(6); i <= 7; i++) {
                final_pos[i] = start.step[i];
                waypoints.back().push_back(final_pos[i]);
            }
        }
    }

    return waypoints;
}

void Real::stabilizeArm(RobotArm arm, std::vector<double> destination, int intensity)
{
    int cpt(0);
    std::vector<double> prev = arm.get_joint_values();

    while (cpt < intensity) {
        std::vector<double> curr = arm.get_joint_values();

        bool ok(true);
        for (int j(0); j < curr.size(); j++) {
            if (fabs(curr[j] - prev[j]) > 1e-5) {
                ok = false;
            }
        }

        if (ok) {
            cpt++;
        }
        prev = curr;
    }
}



