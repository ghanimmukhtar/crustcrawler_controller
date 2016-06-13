#include "../include/arm_controller.hpp"
#include <iostream>


Arm_controller::Arm_controller()
{
    setNbWaypoints(0); // Number of waypoints in the controller
}

Arm_controller::~Arm_controller()
{

}

void Arm_controller::showControllers()
{
    std::cout << "[controller] " << getNbWaypoints() << " waypoints in the trajectory :" << std::endl;
    for (int i(-1); i < getNbWaypoints(); i++) {
        std::cout << "[controller] wp" << i + 1 << " : ";
        for (int j(0); j < getWaypoint(i).step.size(); j++) {
            std::cout << getWaypoint(i).step[j] << " ";
        }
        std::cout << "\t" << getWaypoint(i).fingersValue << std::endl;
    }
}

void Arm_controller::showRealController()
{
    std::cout << "[controller] ";
    for (int i(0); i < getNbWaypoints(); i++) {
        for (int j(0); j < getWaypoint(i).step.size() - 3; j++) {
            std::cout << getWaypoint(i).step[j] << " ";
        }
        std::cout << (getWaypoint(i).step[5] - 0.1) / 0.8 << " ";

        if (getWaypoint(i).step[getWaypoint(i).step.size() - 2] == 0.5) {
            std::cout << getWaypoint(i).fingersValue / 2.0f << " ";
        }
        else {
            std::cout << 0.5 + (getWaypoint(i).fingersValue / 2.0f) << " ";
        }
    }
    std::cout << std::endl;
}

void Arm_controller::addWaypoint(std::vector<double> step)
{
    // To avoid many wrist explosion, has to reduce its rotation to [0.1 - 0.9] (was [0 - 1])
    step[5] *= 0.8;
    step[5] += 0.1;
    double val = step[6];
    if (val >= 0.5) { // If gripper has to open itself
        step[6] = 0.25;
        step.push_back(0.75);
        val -= 0.5;
    }
    else { // If gripper has to close itself
        step[6] = 0.5;
        step.push_back(0.5);
    }
    val *= 2;

    waypoint wp;
    wp.step = step;
    wp.fingersValue = val;
    getTrajectory().push_back(wp);
    setNbWaypoints(getNbWaypoints() + 1);
}

void Arm_controller::adaptToReality()
{
    for (int i(0); i < getNbWaypoints(); i++) {
        waypoint joint_values = getWaypoint(i);

        // Has to be between -0.5 and 0.5 for the real arm
        for (int j(0); j < joint_values.step.size(); j++) {
            joint_values.step[j] -= 0.5f;
        }
        // Plinth rotation has a small gap
        joint_values.step[0] *= -1.0f;
        ///*
        joint_values.step[0] -= 0.015f;
        // Angle of motor 1 (basal motor : double Mx106) is bigger than 180 degrees
        joint_values.step[1] +=
                _sign(joint_values.step[1] * (-1)) * fabs(joint_values.step[1]) * 0.05 * (4.0f / 4.0f) / 0.5f;
        // Wrong angle of last join before wrist
        joint_values.step[4] -= 0.03f;
        //*/
        // Rotation of the real wrist not the same as in simulation
        joint_values.step[5] *= -1.0f;
        // Adapt fingers values
        if (joint_values.step[6] != 0 || joint_values.step[7] != 0) {
            joint_values.step[6] += 0.4;
            joint_values.step[7] -= 0.4f;
        }
        else {
            joint_values.step[6] += 0.33f;
            joint_values.step[7] -= 0.33f;
        }

        setWaypoint(joint_values, i);
    }

    std::vector<double> org;
    for (int i(0); i < 6; i++) {
        org.push_back(0.0f);
    }
    org.push_back(0.08f);
    org.push_back(-0.08f);
    setOrigin(org);
}

double Arm_controller::getMoveTime(waypoint a, waypoint b, double speed)
{
    std::vector<double> distances;
    for (int i(0); i < a.step.size(); i++) {
        distances.push_back(fabs(a.step[i] - b.step[i]));
    }
    double maxDist = *std::max_element(distances.begin(), distances.end());

    return maxDist / speed;
}

double Arm_controller::findVariance()
{
    std::vector<double> variance;
    for (int i(0); i < getNbWaypoints(); i++) {
        std::vector<double> v;
        for (int j(1); j < 5; j++) {
            v.push_back(getWaypoint(i).step[j]);
        }
        variance.push_back(_variance(v));
    }

    return _mean(variance);
}

int Arm_controller::_sign(double a)
{
    if (a < 0) { return -1; }
    else if (a == 0) { return 0; }
    else { return 1; }
}

std::vector<double> Arm_controller::_getVector(double val)
{
    std::vector<double> v;
    for (int i(0); i < 6; i++) {
        v.push_back(val);
    }
    v.push_back(0.25f);
    v.push_back(0.75f);
    return v;
}

double Arm_controller::_variance(std::vector<double> v)
{
    double cpt = 0;
    double mean = _mean(v);
    for (int i(0); i < v.size(); i++) {
        cpt += v[i] * v[i];
    }
    cpt /= double(v.size());
    return cpt - mean * mean;
}

double Arm_controller::_mean(std::vector<double> v)
{
    double cpt = 0;
    for (int i(0); i < v.size(); i++) {
        cpt += v[i];
    }
    return cpt / double(v.size());
}
