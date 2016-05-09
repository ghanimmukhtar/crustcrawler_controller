#ifndef ROBOT_ARM_HORI_HPP
#define ROBOT_ARM_HORI_HPP

#include <robdyn/robot/robot.hh>
#include "environment_arm.hpp"
#include "robot_params.hpp"

namespace robot {
	
	class Arm : public Robot {
	
	public:
	
		Arm(ode::Environment_arm & env, float height);
		~Arm();
		
		
		/**
		 * @brief get the position of the end of the gripper
		 * @return the position
		 */
		Eigen::Vector3d pos() const;
		
		/**
		 * @brief add a segment to the arm
		 * @param i is the index of the segment in the arm
		 * @param the mass of the segment
		 * @param the length of the segment
		 * @param the width of the segment
		 * @param the height of the segment
		 * @param represents the height where to place the segment (the height of the object i-1 of the arm)
		 * @param the environment where the arm is located
		 * @param a parameter indicating the orientation of the servo-motor : 0 is a rotation on x,y plan and 1 a rotation on y,z plan 
		 * @param the environment where the arm is located
		 */
		void add_seg(int i, float mass, float length, float width, float height, float current_height, ode::Environment_arm& env, int dim, int servo_type = 0);
		
		/**
		 * @brief Add a finger to the arm
		 * @param i is the index of the finger in the arm
		 * @param the mass of all the parts of the finger
		 * @param the length of all the parts of the finger
		 * @param the width of all the parts of the finger
		 * @param the height of all the parts of the finger
		 * @param represents the height where to place the finger (the height of the object i-1 of the arm)
		 * @param the environment where the arm is located
		 * @param the distance between the two fingers
		 * @param a boolean : true if did the right finger, false for the left finger
		 */
		void add_finger(int i, float mass[5], float length[5], float width[5], float height[5], float current_height, ode::Environment_arm& env, float inter_dist, bool right);
		
		
		// Getters / Setters
		void setHeight(float height) { _height = height; }
		float getHeight() { return _height; }
		
	 
	private:
	
		float _height;
	};
}
#endif
