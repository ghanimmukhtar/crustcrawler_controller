#include <iostream>
#include <limits>
#include "../include/robotArm.hpp"

#include <fstream>


RobotArm::RobotArm() {
	setCrash(false);
	setErrors(0);
    init();
}

RobotArm::~RobotArm() {
    std::cout << "i m dying" << std::endl;
//	relax();
}

void RobotArm::init(){
    try {


        getController().open_serial(DYNAMIXELSERIAL, B1000000);

        // Scan actuators IDs
        getController().scan_ax12s();

        const std::vector<byte_t>& ax12_ids = getController().ax12_ids();
        if (!ax12_ids.size()) {
            std::cerr << "[real_arm] no dynamixel detected" << std::endl;
            setCrash(true);
            return;
        }
        std::cout << "[real_arm] " << ax12_ids.size() << " dynamixel are connected" << std::endl;

        getActuatorsIds().push_back(1); //turntable : MX-28
        getActuatorsIds().push_back(2); // actuators 2 & 3 : 1st joint : 2*MX-106
        getActuatorsIds().push_back(3);
        getActuatorsIds().push_back(4); // 2nd joint : MX-106
        getActuatorsIds().push_back(5); // 3rd joint : MX-64
        getActuatorsIds().push_back(6); // 4th joint : MX-28
        getActuatorsIds().push_back(7); // 5th joint "base gripper" : MX-28
        getActuatorsIds().push_back(8); // right finger joint : AX-18A
        getActuatorsIds().push_back(9); // left finger joint : AX-18A

        // PID
        /* P coefficient ; cw compliance slope
         * I coefficient ; ccw compliance margin
         * D coefficient ; cw compliance margin
         */
        /*changePValue(82);
        changeIValue(20);
        changeDValue(0);*/
        changePValue(32);
        changeIValue(0);
        changeDValue(0);

    }
    catch (dynamixel::Error e) {
        std::cerr << "[real_arm] error (dynamixel): " << e.msg() << std::endl;
    }

}

void RobotArm::reset() {
	try {
		if(!getController().isOpen()) {
			std::cout << "[real_arm] re-opening dynamixel's serial" << std::endl;
			getController().open_serial(DYNAMIXELSERIAL, B1000000);
		}
		getController().flush();
	}
	catch (dynamixel::Error e) {
		std::cerr << "[real_arm] error (dynamixel): " << e.msg() << std::endl;
		setCrash(true);
		return;
	}

	enable();
}

void RobotArm::relax() {
	try {
		for (size_t i(0) ; i < getActuatorsIds().size() ; ++i) {
			getController().send(dynamixel::ax12::TorqueEnable(getActuatorsIds()[i], false));
			getController().recv(Parameters::read_duration, getStatus());
		}
	} catch(dynamixel::Error e) {
		std::cerr << "[real_arm] dynamixel error : " << e.msg() << std::endl;
	}
}

void RobotArm::relax_gripper(){
    unsigned char grip_m1 = 8; unsigned char grip_m2 = 9;
    try {
        std::cout << "************* i am openning the gripper **************************" << std::endl;
            getController().send(dynamixel::ax12::TorqueEnable(grip_m1, false));
            getController().recv(Parameters::read_duration, getStatus());

            getController().send(dynamixel::ax12::TorqueEnable(grip_m2, false));
            getController().recv(Parameters::read_duration, getStatus());

    } catch(dynamixel::Error e) {
        std::cerr << "[real_arm] dynamixel error : " << e.msg() << std::endl;
    }
}
void RobotArm::enable() {
	try {
		std::cout << "[real_arm] enable motor..." << std::endl;
		for(size_t i(0) ; i < getActuatorsIds().size() ; ++i) {
			getController().send(dynamixel::ax12::TorqueEnable(getActuatorsIds()[i], true));
			getController().recv(Parameters::read_duration, getStatus());
		}
		
		usleep(1e5);
	} catch(dynamixel::Error e) {
		std::cerr << "[real_arm] dynamixel error : " << e.msg() << std::endl;
	}
}

std::vector<float> RobotArm::get_joint_values(std::vector<byte_t> actuators_ids) {
	std::vector<float> current_pos;
	std::vector<int> current_pos_step_per_turn;
	if(actuators_ids.empty())
		actuators_ids = getActuatorsIds();

	for (size_t i(0) ; i < actuators_ids.size() ; i++) {
		try {
			getController().send(dynamixel::ax12::GetPosition(actuators_ids[i]));
			getController().recv(Parameters::read_duration, getStatus());
			current_pos_step_per_turn.push_back(getStatus().decode16());

			if(actuators_ids[i] < 8 && actuators_ids[i] != 2)
				current_pos.push_back(MX_stepperturn_to_rad(current_pos_step_per_turn[i]));
			else if(actuators_ids[i] >= 8)
				current_pos.push_back(AX_stepperturn_to_rad(current_pos_step_per_turn[i]));
		} catch(dynamixel::Error e) {
			std::cerr << "[real_arm] get joint values : error :" << e.msg() << std::endl;
            exit(1);
			return std::vector<float>(1, 100);
		}
	}
		usleep(1e3);

	return current_pos;
}
//For the acutatros defined in actuators_ids vector, returns their speed in RPS (Revolution Per Second)
std::vector<float> RobotArm::get_joint_speeds(std::vector<byte_t> actuators_ids) {
    std::vector<float> current_speed;
    std::vector<int> current_speed_rpm;
    if(actuators_ids.empty())
        actuators_ids = getActuatorsIds();

    for (size_t i(0) ; i < actuators_ids.size() ; i++) {
        try {
            /*dynamixel method that reads the speed, it returns the speed in RPM (Revolution Per Minute), furthermore, the value is interpreted in a certain way: all values from
             * 0 to 1023 represents positive velocities (counter-clockwise rotation), while the values from 1024 to 2047 represents the negative velocities (clockwise) with 1024
             * being the zero in this sense.
            */
            getController().send(dynamixel::ax12::GetSpeed(actuators_ids[i]));
            getController().recv(Parameters::read_duration, getStatus());
            current_speed_rpm.push_back(getStatus().decode16());

            //convert the RPM to RPS with the proper sign ("+" for counter-clockwise and "-" for clockwise rotation)
            if(actuators_ids[i] < 8 && actuators_ids[i] != 2){
                if (current_speed_rpm[i] < 1024)
                    current_speed.push_back(rpm_to_rps(current_speed_rpm[i]));
                else current_speed.push_back(-1*rpm_to_rps(current_speed_rpm[i]-1024));
            }
        } catch(dynamixel::Error e) {
            std::cerr << "[real_arm] get joint values : error :" << e.msg() << std::endl;
            exit(1);
            return std::vector<float>(1, 100);
        }
    }
        usleep(1e3);

    return current_speed;
}

//get the load of each joint
std::vector<float> RobotArm::get_joint_loads(std::vector<byte_t> actuators_ids) {
    std::vector<float> current_load;
    std::vector<int> current_load_integer;
    if(actuators_ids.empty())
        actuators_ids = getActuatorsIds();

    for (size_t i(0) ; i < actuators_ids.size() ; i++) {
        try {
            getController().send(dynamixel::ax12::GetLoad(actuators_ids[i]));
            getController().recv(Parameters::read_duration, getStatus());
            current_load_integer.push_back(getStatus().decode16());

            if(actuators_ids[i] < 8) {
                if(current_load_integer[i] < 1024)
                    current_load.push_back(MX_integer_load_to_float(current_load_integer[i]));
                else
                    current_load.push_back(MX_integer_load_to_float(current_load_integer[i] - 1024));
            }
        } catch(dynamixel::Error e) {
            std::cerr << "[real_arm] get joint values : error :" << e.msg() << std::endl;
            exit(1);
            return std::vector<float>(1, 100);
        }
    }
        usleep(1e3);

    return current_load;
}

//set joints position (angle) to values
bool RobotArm::set_joint_values(std::vector<float> values, std::vector<byte_t> actuators_ids) {
	if(actuators_ids.empty())
		actuators_ids = getActuatorsIds();
	try {
		std::vector<int> pos(actuators_ids.size());

		int i(0);
		for(std::vector<float>::iterator it = values.begin(); it != values.end(); it++) {
			if(actuators_ids[i] < 8) {
				pos[i] = rad_to_stepperturn_MX(*it);
				if(actuators_ids[i] == 2) {
					pos[i+1] = pos[i];
					it = values.insert(it,pos[i]);
					i++;
					it++;
				}
			} else {
				pos[i]=rad_to_stepperturn_AX(*it);
			}

			i++;
		}

        getController().send(dynamixel::ax12::SetPositions(actuators_ids, pos));
        getController().recv(Parameters::read_duration, getStatus());


		
		usleep(1e4);

		return true;

	} catch(dynamixel::Error e) {
		std::cerr << "[real_arm] set_joint_values : dynamixel error : " << e.msg() << std::endl;
		return false;
	}
}

//Sets the speed of the joints defined in actuators_ids to the corresponding velocities given in values
bool RobotArm::set_joint_speeds(std::vector<float> values, std::vector<byte_t> actuators_ids) {

    if(actuators_ids.empty())
        actuators_ids = getActuatorsIds();
    try {
        float speed_value;
        std::vector<int> vel(actuators_ids.size());
        std::vector<bool> dir(actuators_ids.size());

        int i(0);
        for(std::vector<float>::iterator it = values.begin(); it != values.end(); it++) {
                //convert the speed from rps to rpm
                speed_value = rps_to_rpm(*it);
                /*set safety limits for speed (it is set arbitrary to 100 RPM but the user can change it as needed). An important note is that the duration of the trajectory
                 * significant role in determining the maximum joint velocity the algorithm will give. So if the returned velocities are higher than the set safety limits then
                 * just try to prolong the duration the duration
                */
                if (speed_value > 200){
                    vel[i] = 200;

                }
                else
                    vel[i] = rps_to_rpm(*it);

                //set the direction: if minus then it is clockwise and direction should be true, otherwise it is false and it is counter_clockwise
                if(*it < 0){
                    dir[i] = true;
                } else {
                    dir[i] = false;
                }

                //the second and third joints have the same speed/position, they are basically the same because they are interconnected physically
                if(actuators_ids[i] == 2) {
                    vel[i+1] = vel[i];
                    dir[i+1] = dir[i];
                    it = values.insert(it,vel[i]);
                    i++;
                    it++;
                }

            i++;
        }

        //dynamixel method that sets the speeds of motors defined in "actuators_ids", with velocities given in "vel", and the directions described in "dir"
        getController().send(dynamixel::ax12::SetSpeeds(actuators_ids, vel,dir));
        getController().recv(Parameters::read_duration, getStatus());

        usleep(1e4);

        return true;

    } catch(dynamixel::Error e) {
        std::cerr << "[real_arm] set_joint_values : dynamixel error : " << e.msg() << std::endl;
        return false;
    }
}

bool RobotArm::set_speeds_to_zero(std::vector <byte_t> actuators_ids)
{
    std::vector<float> joints_speeds(actuators_ids.size(), 0.0);
    return set_joint_speeds(joints_speeds, actuators_ids);
}

//set the operation mode for given actuators to wheel mode which allows their control in velocity
bool RobotArm::mode_speed(std::vector<byte_t> reduced_actuator_id){
    try{
        for(int i = 0;i<reduced_actuator_id.size();i++){
            getController().send(dynamixel::ax12::SetContinuous(reduced_actuator_id[i]));
            getController().recv(Parameters::read_duration, getStatus());
            usleep(1e4);
        }
        return true;
    }catch(dynamixel::Error e) {
        std::cerr << "[real_arm] set mode to speed : dynamixel error : " << e.msg() << std::endl;
        return false;
    }
}

//set the operation mode for given actuators to joint mode which allows their control in position
bool RobotArm::mode_position(std::vector<byte_t> actuators_ids){
    try{
        for(int i = 0;i<actuators_ids.size();i++){
            /*dynamixel method that take care of changing the joint mode to position (joint) mode for joints defined in "actuators_ids". In the dynamixel library that belongs to
             * a git commit equal to or before the "commit 0c7be73bbc57148b3369345dc59a126ad3aefc64", this method is defined in file "ax12.hpp" which usually should resides in
             * "/usr/local/include/dynamixel", given that the library has been installed, obviously. Now the important point is that if the motors that you want to set in joint
             * mode belong to MX family, then you need to modify this method to show that their range actually goes from 0 to 4095 rather than the range 0 to 1023 which belongs
             * to motors from the AX family, so basically the line stating: params.push_back(3); should be changed to: params.push_back(15);
            */
            getController().send(dynamixel::ax12::UnsetContinuous(actuators_ids[i]));
            getController().recv(Parameters::read_duration, getStatus());
            usleep(1e4);
        }
        return true;
    }catch(dynamixel::Error e) {
        std::cerr << "[real_arm] set mode to position : dynamixel error : " << e.msg() << std::endl;
        return false;
    }
}
void RobotArm::changePValue(int val, int servo_index) {
	if(servo_index == -1) {
		for(int i(2) ; i < 7 ; i++) {
			getController().send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_slope, val));
			getController().recv(Parameters::read_duration, getStatus());
		}
	} else if(servo_index >= 2 && servo_index <= 6) {
		getController().send(dynamixel::ax12::WriteData(servo_index, dynamixel::ax12::ctrl::cw_compliance_slope, val));
		getController().recv(Parameters::read_duration, getStatus());
	}
}

void RobotArm::changeIValue(int val, int servo_index) {
	if(servo_index == -1) {
		for(int i(2) ; i < 7 ; i++) {
			getController().send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::ccw_compliance_margin, val));
			getController().recv(Parameters::read_duration, getStatus());
		}
	} else if(servo_index >= 2 && servo_index <= 6) {
		getController().send(dynamixel::ax12::WriteData(servo_index, dynamixel::ax12::ctrl::ccw_compliance_margin, val));
		getController().recv(Parameters::read_duration, getStatus());
	}
}

void RobotArm::changeDValue(int val, int servo_index) {
	if(servo_index == -1) {
		for(int i(2) ; i < 7 ; i++) {
			getController().send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_margin, val));
			getController().recv(Parameters::read_duration, getStatus());
		}
	} else if(servo_index >= 2 && servo_index <= 6) {
		getController().send(dynamixel::ax12::WriteData(servo_index, dynamixel::ax12::ctrl::cw_compliance_margin, val));
		getController().recv(Parameters::read_duration, getStatus());
	}
}

void RobotArm::close_usb_controllers() {
	getController().close_serial();  
}


