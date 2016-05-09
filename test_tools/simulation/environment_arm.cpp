#include <iostream>
#include <Eigen/Geometry>
#include "environment_arm.hpp"
#include "robdyn/ode/object.hh"
#include "robdyn/ode/box.hh"

namespace ode {
	
	Environment_arm::Environment_arm() : _internal_collision(false) {
		_init(true);
		setTableTouched(false);
	}
	Environment_arm::Environment_arm(float angle) : _internal_collision(false) {
		_init(true, angle);
		setTableTouched(false);
	}
	Environment_arm::Environment_arm(const Environment_arm& env) : _internal_collision(false) {
		_init(true, env.angle);
		_arm_objects = env._arm_objects;
		setTableTouched(false);
	}
	Environment_arm::Environment_arm::~Environment_arm() {}
	
	void Environment_arm::add_table(ode::Environment_arm& env, float tableLength, float tableWidth, float tableHeight, Eigen::Vector3d pos) {
		Object::ptr_t p(new Box(env, pos, 20, tableLength, tableWidth, tableHeight));
		getTable().push_back(p);
		p->fix();
		env.add_env_object(*p);
	}
	
	void Environment_arm::add_cube(Environment_arm& env, Eigen::Vector3d pos, float size, float mass, float rotation) {
		pos[2] += size/2;
		Object::ptr_t p(new Box(env, pos, mass, size, size, size));
		dMatrix3 R;
		dRFromAxisAndAngle(R, 0, 0, 1, rotation);
		dBodySetRotation(p->get_body(), R);
		getCube().push_back(p);
		env.add_env_object(*p);
	}

	void Environment_arm::add_basket(int i, Environment_arm& env, Eigen::Vector3d pos, float length, float width, float height, float thickness) {
		static const double mass(0.2);
		std::vector<Object::ptr_t> p;
		
		pos[2] += thickness/2;
		p.push_back((Object::ptr_t) new Box(env, pos, mass, length-thickness, width-thickness, thickness));
		
		pos[0] += -(length/2)+(thickness/2);
		pos[2] += (height/2)-(thickness/2);
		p.push_back((Object::ptr_t) new Box(env, pos, mass, thickness, width, height));
		
		pos[0] += length-thickness;
		p.push_back((Object::ptr_t) new Box(env, pos, mass, thickness, width, height));
		
		pos[0] += -(length/2)+(thickness/2);
		pos[1] += -(width/2)+(thickness/2);
		p.push_back((Object::ptr_t) new Box(env, pos, mass, length-(2*thickness), thickness, height));
		
		pos[1] += width-thickness;
		p.push_back((Object::ptr_t) new Box(env, pos, mass, length-(2*thickness), thickness, height));
		
		std::vector<ode::Object::ptr_t> bskt;
		for(int i(0) ; i < 5 ; i++) {
			bskt.push_back(p[i]);
			p[i]->fix();
			env.add_env_object(*p[i]);
		}
		
		getBaskets().push_back(bskt);
	}
	
	void Environment_arm::accept_table(ode::ConstVisitor &v) {
		v.visit(getTable());
	}
	
	void Environment_arm::accept_cube(ode::ConstVisitor &v) {
		v.visit(getCube());
	}
	
	void Environment_arm::accept_basket(ode::ConstVisitor &v, int i) {
		v.visit(getBasket(i));
	}
	
	void Environment_arm::add_env_object(ode::Object& o) {
		getEnvObjects().insert(o.get_geom());
	}
	
	void Environment_arm::add_arm_object(int seg, ode::Object& o) {
		getArmObjects().insert(std::pair<int, dGeomID> (seg, o.get_geom()));
	}
	
	Eigen::Vector3d Environment_arm::getCubePos() {
		return getCube()[getCube().size()-1]->get_pos();
	}
	
	void Environment_arm::_collision(dGeomID o1, dGeomID o2) {
		int g1(_ground_objects.find(o1) != _ground_objects.end()); // true if o1 is a ground object
		int g2(_ground_objects.find(o2) != _ground_objects.end()); // true if o2 is a ground object

		int p1(-1);
		int p2(-1);


		if (!(g1 ^ g2)) { // object of the same kind (2 ground, or 2 arm/env objects)
			if(_arm_objects.find(std::pair<int,dGeomID>(0,o1)) != _arm_objects.end()) // if o1 is a part of the arm
				p1 = _arm_objects.find(std::pair<int,dGeomID>(0,o1))->first; // p1 take the index value of the arm part
			else if(_env_objects.find(o1) != _env_objects.end()) { // if o1 is a part of the environment
				if(_env_objects.find(o1) == _env_objects.find(getTable()[0]->get_geom())) // if o1 is the table
					p1 = 100;
				else
					p1 = 42;
			} else // unknown object																		
				return;

			if(_arm_objects.find(std::pair<int,dGeomID>(0,o2)) != _arm_objects.end()) // if o2 is a part of the arm
				p2 = _arm_objects.find(std::pair<int,dGeomID>(0,o2))->first; // p2 take the index value of the arm part
			else if(_env_objects.find(o2) != _env_objects.end()) { // if o2 is a part of the environment
				if(_env_objects.find(o2) == _env_objects.find(getTable()[0]->get_geom())) // if o2 is the table
					p2 = 100;
				else
					p2 = 84;
			} else // unknown object																		 
				return;

			if(p1 > 1 && p2 > 1 && abs(p1-p2) <= 1)
				return;
		}

		if((g1 == 1 && p2 == 100) || (g2 == 1 && p1 == 100)) //collision between table and ground useless			 
			return;

		
		if((p1 == 100 && p2 < 42 && p2 != 0) // IF o1 is the table AND o2 is part of the arm AND o2 is not the plinth
			|| (p2 == 100 && p1 < 42 && p1 != 0)) // IF o2 is the table AND o1 is part of the arm AND o1 is not the plinth
			setTableTouched(true);
		
		static const int N(10);
		int n;
		dContact contact[N];
		n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	
		if (n > 0) {
			if (!(g1 ^ g2) && !(_env_objects.find(o1) != _env_objects.end() || _env_objects.find(o2) != _env_objects.end())) {
				_internal_collision = true;
				return;
			}

			dBodyID b(0);
			if(g1 == -1) { //so g2 is in contact with the ground
				b = dGeomGetBody(o2);
			} else { //so g1 in in contact with the ground
				b = dGeomGetBody(o1);
			}
		
			Object*o = (Object *)dBodyGetData(b);
			if (dBodyGetData(b))
				o->set_in_contact(true);
			
			for (int i(0) ; i < n ; i++) {
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact[i].surface.mu = 0.8; //dInfinity; //0.8
				contact[i].surface.slip1 = 0.001; // 0.01;
				contact[i].surface.slip2 = 0.1; //0.01;
				contact[i].surface.soft_erp = 0.1; //0.1
				contact[i].surface.soft_cfm = 0.00001; //penetration/softness
			
				dJointID c = dJointCreateContact(get_world(), get_contactgroup(), &contact[i]);
				dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
			}
		}
	}
} //namespace ode
