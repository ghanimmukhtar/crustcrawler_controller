#include "arm_crustcrawler.hpp" 
#include "robdyn/ode/box.hh"
#include "robdyn/ode/capped_cyl.hh"
#include "robdyn/ode/sphere.hh"
#include "robdyn/ode/motor.hh"
#include "robdyn/ode/servodynamixel.hh"
#include "robdyn/ode/servo.hh"
#include "robdyn/ode/ax12.hh"


using namespace ode;

namespace robot {
	
	Arm::Arm(ode::Environment_arm & env, double height) {
		setHeight(height);
		
		/// creation of robot's body
		_main_body = Object::ptr_t(new Box(env, Eigen::Vector3d(0, 0, height+Params::body_height/2),
							 Params::body_mass, Params::body_length, Params::body_width, Params::body_height));
		
		height += Params::body_height;
		_main_body->fix(); 
		_bodies.push_back(_main_body);
		env.add_arm_object(0, *_main_body);
		
		/// first part
		add_seg(1,	Params::p1_mass, Params::p1_length,	Params::p1_width, Params::p1_height, height, env, 0, 3);
		height += Params::p1_height;
		
		add_seg(2,	Params::p2_mass, Params::p2_length,	Params::p2_width, Params::p2_height, height, env, 1, 5);
		height += Params::p2_height;
		
		add_seg(3,	Params::p3_mass, Params::p3_length,	Params::p3_width, Params::p3_height, height, env, 1, 4);
		height += Params::p3_height;
		
		add_seg(4,	Params::p4_mass, Params::p4_length,	Params::p4_width, Params::p4_height, height, env, 1, 3);
		height += Params::p4_height;
		
		add_seg(5,	Params::p5_mass, Params::p5_length,	Params::p5_width, Params::p5_height, height, env, 1, 2);
		height += Params::p5_height;
		
		add_seg(6,	Params::p6_mass, Params::p6_length,	Params::p6_width, Params::p6_height, height, env, 0, 2);
		height += Params::p6_height;
		
		double fmass[5], flength[5], fwidth[5], fheight[5];
		fwidth[0] = Params::finger_width1;
		fwidth[1] = Params::finger_width2;
		fwidth[2] = Params::finger_width3;
		fwidth[3] = Params::finger_width4;
		fwidth[4] = Params::finger_width5;
		fheight[0] = Params::finger_height1;
		fheight[1] = Params::finger_height2;
		fheight[2] = Params::finger_height3;
		fheight[3] = Params::finger_height4;
		fheight[4] = Params::finger_height5;
		flength[0] = Params::finger_length1;
		flength[1] = Params::finger_length2;
		flength[2] = Params::finger_length3;
		flength[3] = Params::finger_length4;
		flength[4] = Params::finger_length5;
		fmass[0] = Params::finger_mass1;
		fmass[1] = Params::finger_mass2;
		fmass[2] = Params::finger_mass3;
		fmass[3] = Params::finger_mass4;
		fmass[4] = Params::finger_mass5;
		
		add_finger(7, fmass, flength, fwidth, fheight, height, env, Params::finger_inter_dist, false);
		add_finger(8, fmass, flength, fwidth, fheight, height, env, Params::finger_inter_dist, true);		
		height += Params::finger_height;
	}
	
	Arm::~Arm() {
		
	}
	
	Eigen::Vector3d Arm::pos() const {
		return (_bodies[_bodies.size()-1]->get_pos()+_bodies[_bodies.size()-6]->get_pos())/2;
	}
	
	void Arm::add_seg(int i, double mass, double length, double width, double height, double current_height, Environment_arm& env, int dim, int servo_type) {
		Object::ptr_t p(new Box(env, Eigen::Vector3d(0, 0, current_height + height/2), mass, length, width, height));
		
		_bodies.push_back(p);
		env.add_arm_object(i, *p);
		
		assert(_bodies.size() >= 2);
		
		Servo::ptr_t s1;
		switch(servo_type) {
		case 1:
			s1 = Servo::ptr_t(new Ax12(env, Eigen::Vector3d(0,
									0,
									current_height),
				 *_bodies[_bodies.size()-2], *p));
			break;
			
		case 2:
			s1 = Servo::ptr_t(new Mx28(env, Eigen::Vector3d(0,
									0,
									current_height),
				 *_bodies[_bodies.size()-2], *p));
			break;
			
		case 3:
			s1 = Servo::ptr_t(new Mx64(env, Eigen::Vector3d(0,
									0,
									current_height),
				 *_bodies[_bodies.size()-2], *p));
			break;

		case 4:
			s1 = Servo::ptr_t(new Mx106(env, Eigen::Vector3d(0,
									 0,
									 current_height),
					*_bodies[_bodies.size()-2], *p));
			break;

		case 5:
			s1 = Servo::ptr_t(new DoubleMx106(env, Eigen::Vector3d(0,
									 0,
									 current_height),
					*_bodies[_bodies.size()-2], *p));
			break;

		default:
			s1 = Servo::ptr_t(new Servo(env, Eigen::Vector3d(0,
									 0,
									 current_height),
					*_bodies[_bodies.size()-2], *p));
			break;
		}

		
		if(dim == 0) {
            s1->set_axis(0, Eigen::Vector3d(0, 0, -1));
			s1->set_axis(2, Eigen::Vector3d(1, 0, 0));
		} else {
			s1->set_axis(0, Eigen::Vector3d(0, 1, 0));
			s1->set_axis(2, Eigen::Vector3d(1, 0, 0));
		}
		
		_servos.push_back(s1);
	}
	 
	void Arm::add_finger(int i, double mass[5], double length[5], double width[5], double height[5], double current_height, Environment_arm& env, double inter_dist, bool right) {
		int sign(1);
		if(!right) { sign *= -1; }
		
		double tmp_height(current_height);
		double dist1(((Params::finger_inter_dist/2.0f)+(width[1]/2.0f))*sign);
		double dist2(((Params::finger_inter_dist/2.0f)+width[1]-(width[2]/2.0f))*sign);
		double dist_serv((width[0]/2.0f)*sign);
		
		double angle1(0.12);
		double angle2(0.06);
		
		double curr_dist_y(0);
		std::vector<Object::ptr_t> p;
		
		for(int j(0) ; j < 5 ; j++) {
			if(j == 0) curr_dist_y = 0;
			else if(j == 1) curr_dist_y = dist1;
			else curr_dist_y = dist2;
			
			if(j == 3) { curr_dist_y -= 0.01*sign; tmp_height -= 0.0025; }
			else if(j == 4) { curr_dist_y -= 0.02*sign; tmp_height -= 0.005; }
			
			p.push_back((Object::ptr_t) new Box(env, Eigen::Vector3d(0,
							curr_dist_y,
							tmp_height+height[j]/2),
					 mass[j], length[j], width[j], height[j]));
			
			if(j == 3) { curr_dist_y += 0.01*sign; tmp_height += 0.0025; }
			else if(j == 4) { curr_dist_y += 0.02*sign; tmp_height += 0.005; }
			
			tmp_height += height[j];
			_bodies.push_back(p[j]);
			env.add_arm_object(i, *p[j]);
		}
		
		tmp_height = current_height;
		
		int x;
		if(!right) { x = 7; }
		else { x = 12; }
		
		dMatrix3 R;
		dRFromAxisAndAngle(R, 1, 0, 0, angle1*M_PI*sign);
		dBodySetRotation(_bodies[x+3]->get_body(), R);
		dRFromAxisAndAngle(R, 1, 0, 0, angle2*M_PI*sign);
		dBodySetRotation(_bodies[x+4]->get_body(), R);
		
		
		
		assert(_bodies.size()>=5);
		
		dJointID c1, c2, c3, c4;
		
		c1 = dJointCreateFixed(env.get_world(), 0);
		dJointAttach(c1, _bodies[6]->get_body(), _bodies[x]->get_body());
		//dJointEnable(c1);
		dJointSetFixed(c1);
		tmp_height += height[0];
		
		Servo::ptr_t s = (Servo::ptr_t) new Mx28(env, Eigen::Vector3d(0,
							dist_serv,
							tmp_height),
					 *_bodies[x], *_bodies[x+1]);
		s->set_axis(0, Eigen::Vector3d(1, 0, 0));
		s->set_axis(2, Eigen::Vector3d(0, 0, 1));
		s->set_angle(0, (M_PI*(0.25)*sign));
		_servos.push_back(s);
		
		c2 = dJointCreateFixed(env.get_world(), 0);
		dJointAttach(c2, _bodies[x+1]->get_body(), _bodies[x+2]->get_body());
		dJointSetFixed(c2);
		
		c3 = dJointCreateFixed(env.get_world(), 0);
		dJointAttach(c3, _bodies[x+2]->get_body(), _bodies[x+3]->get_body());
		dJointSetFixed(c3);
		
		c4 = dJointCreateFixed(env.get_world(), 0);
		dJointAttach(c4, _bodies[x+3]->get_body(), _bodies[x+4]->get_body());
		dJointSetFixed(c4);
	}
}


