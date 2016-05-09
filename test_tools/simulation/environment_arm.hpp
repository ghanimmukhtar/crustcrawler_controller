#ifndef ENVIRONMENT_ARM_HH_
#define ENVIRONMENT_ARM_HH_

#include <iostream>
#include <ode/ode.h>
#include <ode/common.h>
#include <set>
#include <robdyn/ode/misc.hh>
#include <robdyn/ode/object.hh>
#include <robdyn/ode/environment.hh>

namespace ode {
	
	struct classcomp {
		bool operator() (const std::pair<int, dGeomID>& lhs, const std::pair<int, dGeomID>& rhs) const {
			return lhs.second < rhs.second;
		}
	};
	
	class Environment_arm: public Environment {
			
	public:
			
		Environment_arm();
		Environment_arm(float angle);
		Environment_arm(const Environment_arm& env);
		~Environment_arm();
		
		
		/**
		 * @brief add the table to the environment
		 * @param the environment
		 * @param length of the table
		 * @param width of the table
		 * @param height of the table
		 * @param position of the table
		 */
		void add_table(ode::Environment_arm& env, float tableLength, float tableWidth, float tableHeight, Eigen::Vector3d pos);
		
		/**
		 * @brief add the cube to the environment
		 * @param the environment
		 * @param position of the cube
		 * @param size of the cube
		 * @param mass of the cube
		 * @param rotation of the cube
		 */
		void add_cube(ode::Environment_arm& env, Eigen::Vector3d pos, float size, float mass, float rotation);
		
		/**
		 * @brief add the baskets to the environment
		 * @param the environment
		 * @param position of the center of all baskets
		 * @param length of a basket
		 * @param width of a basket
		 * @param height of a basket
		 * @param thickness of a basket
		 */
		void add_basket(int i, ode::Environment_arm& env, Eigen::Vector3d pos, float length, float width, float height, float thickness);
		
		/**
		 * @brief permits to recover the position of the cube
		 * @return the position as a Vector3d
		 */
		Eigen::Vector3d getCubePos();
		
		/**
		 * @brief add an object to the environment
		 * @param the object to add
		 */
		void add_env_object(ode::Object& o);
		
		/**
		 * @brief add an object to the environment as part of the arm
		 * @param the object to add
		 */
		void add_arm_object(int seg, ode::Object& o);
		
		/**
		 * @brief permits to put the table in our environment ans to interact with other objects
		 */
		void accept_table(ode::ConstVisitor &v);
		
		/**
		 * @brief permits to put the cube in our environment ans to interact with other objects
		 */
		void accept_cube(ode::ConstVisitor &v);
		
		/**
		 * @brief permits to put the basket i in our environment ans to interact with other objects
		 * @param i corresponds to the index of the concerned basket
		 */
		void accept_basket(ode::ConstVisitor &v, int i);
		
		
		// Getters / Setters
		std::set<dGeomID>& getEnvObjects() { return _env_objects; }
		std::set<std::pair<int, dGeomID>, classcomp>& getArmObjects() { return _arm_objects; }
		
		std::vector<ode::Object::ptr_t>& getTable() { return _table; }
		void setTable(std::vector<ode::Object::ptr_t> table) { _table = table; }
		std::vector<ode::Object::ptr_t>& getCube() { return _cube; }
		void setCube(std::vector<ode::Object::ptr_t> cube) { _cube = cube; }
		std::vector< std::vector<ode::Object::ptr_t> >& getBaskets() { return _baskets; }
		void setBaskets(std::vector< std::vector<ode::Object::ptr_t> > baskets) { _baskets = baskets; }
		std::vector<ode::Object::ptr_t>& getBasket(int i) { return _baskets[i]; }
		void setBskt(std::vector<ode::Object::ptr_t> basket, int i) { _baskets[i] = basket; }
		
		bool getTableTouched() { return _tableTouched; }
		void setTableTouched(bool tableTouched) { _tableTouched = tableTouched; }
		
		
	private:
		
		void _collision(dGeomID o1, dGeomID o2);
		
		bool _internal_collision;
		
		std::set<std::pair<int, dGeomID>, classcomp> _arm_objects;
		std::set<dGeomID> _env_objects;
		
		std::vector<ode::Object::ptr_t> _table;
		std::vector<ode::Object::ptr_t> _cube;
		std::vector< std::vector<ode::Object::ptr_t> > _baskets;
		
		bool _tableTouched;
		
	};
}

#endif


