#include <iostream>
#include <limits>
#include <robotArm.hpp>

#include <fstream>

using namespace real;

void RobotArm :: init()
{
    try
    {
        _controller.open_serial(DYNAMIXELSERIAL,B1000000);

        // Scan actuators IDs
        _controller.scan_ax12s();
        const std::vector<byte_t>& ax12_ids = _controller.ax12_ids();
        if (!ax12_ids.size())
        {
            std::cerr<<"[dynamixel] no dynamixel detected"<<std::endl;
            return;
        }
        std::cout << "[dynamixel] " << ax12_ids.size()
                  << " dynamixel are connected" << std::endl;

        _actuators_ids.push_back(1); //turntable : MX-28
        _actuators_ids.push_back(2); // actuators 2 & 3 : 1st joint : 2*MX-106
        _actuators_ids.push_back(3);
        _actuators_ids.push_back(4); // 2nd joint : MX-106
        _actuators_ids.push_back(5); // 3rd joint : MX-64
        _actuators_ids.push_back(6); // 4th joint : MX-28
        _actuators_ids.push_back(7); // 5th joint "base gripper" : MX-28
        _actuators_ids.push_back(8); // right finger joint : AX-18A
        _actuators_ids.push_back(9); // left finger joint : AX-18A


        std::cout << "initialisation completed" << std::endl;
        for(int i(1) ; i <= 2 ; i++) {
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::ccw_compliance_slope, 16));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::ccw_compliance_margin, 8));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_margin, 15));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_slope, 16));
                _controller.recv(Params::read_duration, _status);
        }

        for(int i(3) ; i < 6 ; i++) {
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::ccw_compliance_slope, 16));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::ccw_compliance_margin, 2));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_margin, 15));
                _controller.recv(Params::read_duration, _status);
                _controller.send(dynamixel::ax12::WriteData(i, dynamixel::ax12::ctrl::cw_compliance_slope, 16));
                _controller.recv(Params::read_duration, _status);
        }
    }
    catch (dynamixel::Error e)
    {
        std::cerr << "error (dynamixel): " << e.msg() << std::endl;
    }



}


void RobotArm :: reset()
{
    try
    {
        if(_controller.isOpen()==false)
        {
            std::cout<<"re-opening dynamixel's serial"<<std::endl;
            _controller.open_serial(DYNAMIXELSERIAL,B1000000);
        }
        _controller.flush();
    }
    catch (dynamixel::Error e)
    {
        std::cerr << "error (dynamixel): " << e.msg() << std::endl;
    }

    std::cout << "setting all dynamixel to zero" << std::endl;
    enable();

    std::vector<float> reset_joint_values;
    for(int i = 0; i < 6; i++){
        reset_joint_values.push_back(0);
    }
    reset_joint_values.push_back(0.15f);
    reset_joint_values.push_back(-0.15f);
    set_joint_trajectory(reset_joint_values);
    _is_close = false;

    std::cout << "done" << std::endl;

}

void RobotArm::open_gripper(){
    std::vector<float> pos(2);

    std::cout << "Open the gripper" << std::endl;

    std::vector<byte_t> actuators_ids(2);
    actuators_ids[0] = 8;
    actuators_ids[1] = 9;

    pos[0] = 0;
    pos[1] = 0;
    if(_is_close){
        if(set_joint_trajectory(pos,actuators_ids))
            _is_close = false;
    }
}

void RobotArm::close_gripper(){
    std::vector<float> pos(2);

    std::cout << "Close the gripper" << std::endl;

    std::vector<byte_t> actuators_ids(2);
    actuators_ids[0] = 8;
    actuators_ids[1] = 9;

    pos[0] = 0.32;
    pos[1] = -0.32;

    if(!_is_close){
        if(set_joint_trajectory(pos,actuators_ids))
            _is_close = true;
    }
}

void RobotArm::toSleepPosition(){
    std::vector<float> pos(8);

    std::cout << "go to sleep position" << std::endl;

    pos[0] = 0.16;
    pos[1] = 0.18;
    pos[2] = 0.31;
    pos[3] = 0.57;
    pos[4] = -0.07;
    pos[5] = 0;
    pos[6] = .32;
    pos[7] = -.32;

    set_joint_trajectory(pos);

}


std::vector<float> RobotArm::get_joint_values(std::vector<byte_t> actuators_ids){

    std::vector<float> current_pos;
    std::vector<int> current_pos_step_per_turn;
    if(actuators_ids.empty())
        actuators_ids = _actuators_ids;

    for (size_t i = 0; i < actuators_ids.size(); i++)
    {
        try{
            _controller.send(dynamixel::ax12::GetPosition(actuators_ids[i]));
            _controller.recv(Params::read_duration, _status);
            current_pos_step_per_turn.push_back(_status.decode16());

            if(actuators_ids[i] < 8 && actuators_ids[i] != 2)
                current_pos.push_back(MX_stepperturn_to_rad(current_pos_step_per_turn[i]));
            else if(actuators_ids[i] >= 8)
                current_pos.push_back(AX_stepperturn_to_rad(current_pos_step_per_turn[i]));

        }catch(dynamixel::Error e){
            std::cerr << "get joint values : error :" << e.msg() << std::endl;
            return std::vector<float>(1,100);
        }
    }

//    current_pos.push_back(MX_stepperturn_to_rad(current_pos_step_per_turn[0]));
//    current_pos.push_back(MX_stepperturn_to_rad(current_pos_step_per_turn[1]));

//    for(int i = 3; i < _actuators_ids.size() - 2; i++)
//        current_pos.push_back(MX_stepperturn_to_rad(current_pos_step_per_turn[i]));
//    for(int i = _actuators_ids.size() - 2; i < _actuators_ids.size(); i++)
//        current_pos.push_back(AX_stepperturn_to_rad(current_pos_step_per_turn[i]));


    return current_pos;

}


bool RobotArm::set_joint_trajectory(std::vector<float> &ctrl,std::vector<byte_t> actuators_ids, bool inverse){
    if(actuators_ids.empty())
        actuators_ids = _actuators_ids;



    std::vector<std::vector<float> > waypoints;
    if(inverse){
        std::cout << "ik control" << std::endl;
        waypoints = control_inverse(ctrl);
        actuators_ids.clear();
        actuators_ids = std::vector<byte_t>(7);
        for(int i = 0; i < 7; i++)
            actuators_ids[i] = _actuators_ids[i];
    }
    else {
        std::cout << "direct control" << std::endl;
        waypoints = getWayPoints(ctrl, actuators_ids);
    }

    for(int i = 0; i < waypoints.size(); i++){
        //        std::cout << "_______________actuators_ids__________________________________________________" << std::endl;
        if(!set_joint_values(waypoints[i],actuators_ids))
            return false;


        //        std::cout << "position " << i << " reached : ";
        //        for(int i = 0 ; i < actual_pos.size() ; i++){
        //            std::cout << actual_pos[i] << "*PI ";
        //        }
        //        std::cout << std::endl;

    }

    return true;
}
bool RobotArm::set_joint_trajectory(std::vector<float> &ctrl, bool inverse){

    std::vector<std::vector<float> > waypoints;
    std::vector<byte_t> actuators_ids = _actuators_ids;
    if(inverse){
        std::cout << "ik control" << std::endl;
        waypoints = control_inverse(ctrl);
        actuators_ids.clear();
        actuators_ids = std::vector<byte_t>(7);
        for(int i = 0; i < 7; i++)
            actuators_ids[i] = _actuators_ids[i];
    }
    else {
        std::cout << "direct control" << std::endl;
        waypoints = getWayPoints(ctrl, actuators_ids);
    }

    for(int i = 0; i < waypoints.size(); i++){
        //        std::cout << "________________________________________________________________" << std::endl;
        if(!set_joint_values(waypoints[i],actuators_ids))
            return false;


        //        std::cout << "position " << i << " reached : ";
        //        for(int i = 0 ; i < actual_pos.size() ; i++){
        //            std::cout << actual_pos[i] << "*PI ";
        //        }
        //        std::cout << std::endl;

    }

    return true;
}
bool RobotArm::set_joint_values(std::vector<float> values,std::vector<byte_t> actuators_ids)
{

    if(actuators_ids.empty())
        actuators_ids = _actuators_ids;
//    for(int i = 0; i < values.size(); i++){
//        assert(values[i] <= .5);
//        assert(values[i] >= -.5);
//      }

    try{
        std::vector<int> pos(actuators_ids.size());

        int i = 0;
        for(std::vector<float>::iterator it = values.begin(); it != values.end(); it++){
            if(actuators_ids[i] < 8){
                pos[i] = rad_to_stepperturn_MX(*it);
                if(actuators_ids[i] == 2){
                    pos[i+1] = pos[i];
                    it = values.insert(it,pos[i]);
                    i++;
                    it++;
                }
            }else{
                pos[i]=rad_to_stepperturn_AX(*it);
            }
            i++;

        }

        _controller.send(dynamixel::ax12::SetPositions(actuators_ids, pos));
        _controller.recv(Params::read_duration, _status);

        usleep(15000);

        return true;

    }catch(dynamixel::Error e){
        std::cerr << "set_joint_values : dynamixel error : " << e.msg() << std::endl;
        return false;
    }
}

std::vector<float> RobotArm::getDistances(std::vector<float> origin, std::vector<float> ctrl) {
    std::vector<float> distance;
    for(int i(0) ; i < origin.size() ; i++)
        distance.push_back(difference(origin[i], ctrl[i]));
    return distance;
}

float RobotArm::difference(float a, float b){
    if(a < b)
        return b-a;
    if(b < a)
        return a-b;
    return 0;
}
std::vector<std::vector<float> > RobotArm::getWayPoints(std::vector<float> goal_values, std::vector<byte_t> actuator_ids){

    float step = 0.015;

    std::vector<float> start = get_joint_values(actuator_ids);
    std::vector<std::vector<float> > waypoints;
    std::vector<float> distances;
    std::vector<float> final_pos(start.size());
    distances = getDistances(start, goal_values);
    float maxDist = *std::max_element(distances.begin(), distances.end());

    float tps_mvt(maxDist/VMAX);

    std::cout << "start pos: ";
    for(int i = 0 ; i < start.size() ; i++){
        std::cout << start[i] << "*PI ";
    }
    std::cout << std::endl;

    std::cout << "expected pos: ";
    for(int i = 0 ; i < goal_values.size() ; i++){
        std::cout << goal_values[i] << "*PI ";
    }
    std::cout << std::endl;

    float t = 0;
    for (; t <= tps_mvt ; t += step) {
        waypoints.push_back(std::vector<float>());
        for(int i = 0 ; i < start.size() ; i++) {
            final_pos[i] = start[i] + (goal_values[i] - start[i])/tps_mvt*t;
            waypoints.back().push_back(start[i] + (goal_values[i] - start[i])/tps_mvt*t);
        }
    }



    std::cout << "number of waypoints : " << waypoints.size() << std::endl;

    return waypoints;
}

std::vector<float> RobotArm::forward_model(const std::vector<float>& a) const {
    std::vector<float> a_rad;
    for(int i = 0; i < a.size(); i++){
        a_rad.push_back(a[i]*M_PI);
    }
    Eigen::MatrixXd dh=dh_mat(a);

    Eigen::Matrix4d mat=Eigen::Matrix4d::Identity(4,4);
    for(size_t i = 0; i < dh.rows(); i++){
        mat = mat*trmat_dh(dh.row(i));
        //        std::cout << trmat_dh(dh.row(i)) << std::endl;
        //        std::cout << "____________________" << std::endl;

    }

    Eigen::VectorXd v=mat*Eigen::Vector4d(0,0,0,1);
    std::vector<float> res;
    res.push_back(v(0));
    res.push_back(v(1));
    res.push_back(v(2));
    return res;

}


std::vector<std::vector<float> > RobotArm::control_inverse(const std::vector<float>& target){

    std::vector<byte_t> actuators_ids(7);
    for(int i = 0; i < 7; i++)
        actuators_ids[i] = _actuators_ids[i];

    std::vector<float> actual_joint_pos = get_joint_values(actuators_ids);
    for(int i = 0; i < actual_joint_pos.size(); i++)
        actual_joint_pos[i] = actual_joint_pos[i]*M_PI;


    std::vector<float> actual_pos = forward_model(actual_joint_pos);

    std::vector<std::vector<float> > waypoints;

    std::cout << "current pos: ";
    for(int i = 0 ; i < actual_pos.size() ; i++){
        std::cout << actual_pos[i] << " ";
    }
    std::cout << std::endl;

    Eigen::Vector3d er;
    for(int i = 0; i < target.size(); i++)
        er(i) = actual_pos[i] - target[i];
    int counter = 0;
    while((fabs(er(0)) > 1e-3 || fabs(er(1)) > 1e-3 || fabs(er(2)) > 1e-3) && counter < Params::max_iteration){

        Eigen::MatrixXd jt(jacobian(actual_joint_pos).block<3, 6>(0, 0));
        Eigen::FullPivLU<Eigen::MatrixXd> lu(jt * jt.transpose());
        Eigen::VectorXd delta_joint(-Params::gain * jt.transpose() * lu.solve(er));
//        std::cout << delta_joint << std::endl;
//        std::cout << "--" << std::endl;

        if(lu.determinant() < 1e-10)
            std::cout << "SINGULARITY" << std::endl;

        std::vector<float> result;
        for(int i = 0; i < actual_joint_pos.size(); i++){
            if(actual_joint_pos[i]+delta_joint[i] > 0.5*M_PI)
              result.push_back(0.5*M_PI);
            else if(actual_joint_pos[i]+delta_joint[i] < -0.5*M_PI)
              result.push_back(-0.5*M_PI);
            else
              result.push_back(actual_joint_pos[i]+delta_joint[i]);
        }

        actual_pos = forward_model(result);
        actual_joint_pos = result;
        for(int i = 0; i < result.size(); i++){
            result[i] = result[i]/M_PI;
          }
        waypoints.push_back(result);

        for(int i = 0; i < target.size(); i++)
            er(i) = actual_pos[i] - target[i];
        counter++;
    }

    std::cout << "expected final pos: ";
    for(int i = 0 ; i < actual_pos.size() ; i++){
        std::cout << actual_pos[i] << " ";
    }
    std::cout << std::endl;


    if(counter < Params::max_iteration){
        std::cout << "number of waypoints :" << counter << std::endl;
        return waypoints;
    }else{
        std::cerr << "no solution found" << std::endl;
        return std::vector<std::vector<float> >();
    }

}

Eigen::Matrix4d RobotArm::trmat_dh(const Eigen::VectorXd& dh)const {
    Eigen::Matrix4d submat;
    submat<<cos(dh(3)), -sin(dh(3))*cos(dh(1)), sin(dh(3))*sin(dh(1)),  dh(0)*cos(dh(3)),
            sin(dh(3)), cos(dh(3))*cos(dh(1)),  -cos(dh(3))*sin(dh(1)), dh(0)*sin(dh(3)),
            0,          sin(dh(1)),             cos(dh(1)),             dh(2),
            0,          0,                      0,                      1;
    return submat;
}

Eigen::MatrixXd RobotArm::dh_mat(const std::vector<float> &a) const {
    Eigen::MatrixXd dh(6,4);

    double p1 = Params::p1_height;
    double p2 = Params::p2_height;
    double p3 = Params::p3_height;
    double p4 = Params::p4_height;
    double p5 = Params::p5_height;
    double p6 = Params::p6_height;
    double body = Params::body_height;
    double finger = Params::finger_height;

    dh <<   0,  M_PI/2, body + p1,          -a[0],
            p2, 0,      0,                  -a[1]+M_PI/2,
            p3, 0,      0,                  -a[2],
            p4, 0,      0,                  -a[3],
            0,  M_PI/2, 0,                  -a[4]+M_PI/2,
            0,  0,      finger + p5 + p6, -a[5];

    return dh;
}

std::vector<Eigen::Matrix4d> RobotArm::sub_trmat(const std::vector<float>& a) const{
    std::vector<Eigen::Matrix4d> mats;
    Eigen::MatrixXd dh=dh_mat(a);
    Eigen::Matrix4d mat=Eigen::Matrix4d::Identity(4,4);
    for(size_t i=0;i<dh.rows();i++){
        mat=mat*trmat_dh(dh.row(i));
        mats.push_back(mat);
    }

    return mats;
}


Eigen::MatrixXd RobotArm::jacobian(const std::vector<float>& a) const {
    std::vector<Eigen::Matrix4d> mats=sub_trmat(a);
    Eigen::MatrixXd jacob(6,6);

    jacob.col(0).head(3)=Eigen::Vector3d::UnitZ().cross((Eigen::Vector3d)mats[5].col(3).head(3));
    jacob.col(0).tail(3)=Eigen::Vector3d::UnitZ();

    for(int i=0;i<mats.size()-1;i++){
        jacob.col(i+1).head(3) = ((Eigen::Vector3d)mats[i].col(2).head(3)).cross((Eigen::Vector3d)mats[5].col(3).head(3) - (Eigen::Vector3d)mats[i].col(3).head(3));
        jacob.col(i+1).tail(3) = mats[i].col(2).head(3);
    }

    // HACK BECAUSE OF INVERTED JOINT
    //    jacob.row(2) = -jacob.row(2);
    //    jacob.row(3) = -jacob.row(3);
    //    jacob.row(4) = -jacob.row(4);
    //    jacob.row(5) = -jacob.row(5);

        jacob = -jacob;


    return jacob;
}
