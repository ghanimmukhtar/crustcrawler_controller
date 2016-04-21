#include "../include/kinematics.hpp"
#include <iostream>
#include <typeinfo>

/* Global variables:
 * these variables are defined here because they are global besides i had trouble initializing them in the kinematic header file, may be their placement will be changed later
*/
//This defines the euclidean distance between initial_pos and target position for x, y and z coordinates
Eigen::VectorXd distance(3);

/*This defines the angles of each joint upon which the geometric parameters table was built, this means that these angles are the zero position and each joint angle should be
 * measured with reference to them
 * */
float tmp[] = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> initial_joint_values (tmp, tmp + 8);

//These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
unsigned char tmp1[] = {1, 2, 3, 4, 5, 6, 7};
std::vector<unsigned char> reduced_actuator_id (tmp1, tmp1 + 7);


//geometric parameters according to the choosing initial configurations defined by initial_joint_values
Eigen::MatrixXd Kinematics::kk_mat(const std::vector<float>& a) const
{
    //robot description (dimensions of each link and joint)
    float body = 0.05;    float p1 = 0.056;    float p2 = 0.23;    float p3 = 0.155;    float p4 = 0.08;    float p5 = 0.07;    float p6 = 0.021;    float gripper_height = 0.171;
    /* This is the geometric representation of the robot according to Khalil-Kleinfinger notation, the additional colomn is to represent the type of the joint it is 0 for
     * revolute, 1 for translational and 2 for fixed joint, it will be useful later for the dynamic model if needed, for more details refer to : "Modeling and Control of
     * Manipulators Part I: Geometric and Kinematic Models" by Wisama KHALIL [1]
     * */
    Eigen::MatrixXd kk(6,5);
    //---|---sigma---|---alpha_j---|---d_j---|---q_j--------|---r_j---|
    kk <<      0,        0,          0,       a[0],         body+p1,
               0,       M_PI/2,       0,       a[1],           0,
               0,         0,         p2,   a[2]+(M_PI/2),      0,
               0,         0,         p3,       a[3],           0,
               0,         0,         p4,   a[4]-(M_PI/2),      0,
               0,      -M_PI/2,       0,       a[5],          gripper_height+p5+p6;
    return kk;
}


/* So kk is a row in geometric parameter table defined in kk_mat(), so for row kk(j) this function will return the homogeneous transfromation between frame (j-1) and (j).
 * Actually this transformation matrix (Tj) is formulated as follows: Tj = Rot(x,alpha_j)*Trans(x,d_j)*Rot(z,q_j)*Trans(z,r_j). For more details refer to [1]
 * */
Eigen::Matrix4d Kinematics::trmat_kk(const Eigen::VectorXd& kk)const {
    Eigen::Matrix4d submat;
    submat<<           cos(kk(3)),           -sin(kk(3)),           0,             kk(2),
            cos(kk(1))*sin(kk(3)), cos(kk(1))*cos(kk(3)), -sin(kk(1)), -kk(4)*sin(kk(1)),
            sin(kk(1))*sin(kk(3)), sin(kk(1))*cos(kk(3)),  cos(kk(1)),  kk(4)*cos(kk(1)),
                                0,                     0,           0,                 1;
    return submat;
}

//gives all intermediate transformation matrices that will be used to generate the jacobian later
std::vector<Eigen::Matrix4d> Kinematics::sub_trmat(const std::vector<float>& a) const{
    std::vector<Eigen::Matrix4d> mats;
    Eigen::MatrixXd kk=kk_mat(a);
    Eigen::Matrix4d mat=Eigen::Matrix4d::Identity(4,4);
    for(size_t i=0;i<kk.rows();i++){
        //transformation between frame i and i-1, when i is 0 it gives the transformation between frame of first joint (turntable joint) and the world frame (fixed frame)
        mat=mat*trmat_kk(kk.row(i));
        mats.push_back(mat);
    }
    return mats;
}

//the kinematic jacobian is calculated here,
Eigen::MatrixXd Kinematics::jacobian(const std::vector<float>& a) const {
    std::vector<Eigen::Matrix4d> mats=sub_trmat(a);
    Eigen::MatrixXd jacob(6,6);
    for(int i=0;i<mats.size();i++){
        /*the formulation of column (i) is as follows: J(1:3,i) = [a_i x L_i_n], and J(4:6,i) = [a_i], with (a_i) being the third column of the transformation matrix for frame (i)
        (i.e. the vector that express axis of rotation for joint i) and (L_i_n) being the distance vector between frame (i) and the frame of end effector (frame 6 in this particular
        case), this distance vector is calculated by subtracting from the fourth column of the last transformation matrix T6(1:3,4) the one of Ti(1:3,4).
        Refer to [1] for more details.*/
        jacob.col(i).head(3) = ((Eigen::Vector3d)mats[i].col(2).head(3)).cross((Eigen::Vector3d)mats[5].col(3).head(3) - (Eigen::Vector3d)mats[i].col(3).head(3));
        jacob.col(i).tail(3) = mats[i].col(2).head(3);
    }
    return jacob;
}

 /* The output of the forward model is the three position Cartesian coordinates and three angles (Roll, Pitch, Yaw) with the following order of rotations
 * (Rot = Rot(z,Roll)*Rot(y,Pitch)*Rot(x,Yaw)), which gives the following 3x3 rotation matrix:
 * [ cos(Roll)*cos(Pitch),  cos(Roll)*sin(Pitch)*sin(Yaw) - sin(Roll)*cos(Yaw), cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw);
 *   sin(Roll)*cos(Pitch),  sin(Roll)*sin(Pitch)*sin(Yaw) + cos(Roll)*cos(Yaw), sin(Roll)*sin(Pitch)*cos(Yaw) - cos(Roll)*sin(Yaw);
 *            -sin(Pitch),                                 cos(Pitch)*sin(Yaw),                                 cos(Pitch)*cos(Yaw)]
 * Refer to [1] for more details.
 * */
std::vector<float> Kinematics::forward_model(const std::vector<float>& a) const {
    //first construct the geometric parameters matrix as described in Khalil-Kleinfinger notation
    Eigen::MatrixXd kk=kk_mat(a);
    //this will the transformation matrix between each joint frame and subsequent one
    Eigen::Matrix4d mat=Eigen::Matrix4d::Identity(4,4);
    //orientation angles about X, Y and Z
    float Roll;    float Pitch;    float Yaw;

    /* This should give the transformation matrix from the frame 0 (F0) to frame 6 (F6) which is at the end of the last actuator (let's denote it T0T6), so to get the pose of
     * the end effector in the absolute world frame two transformations need to be accounted for: first the transformation from F0 to the world absolute frame (let's call it Fw),
     * we can denote it as TwT0, second the transformation from the frame attached to  the end effector (let's call it Fe) to F6 (T6Te). For the complete description of all
     * frames look at the documentation of the code. Refer to [1] for more details.
     * */
    for(size_t i = 0; i < kk.rows(); i++){
        mat = mat*trmat_kk(kk.row(i));
    }

    //The variable v contains the Cartesian position coordinates for the given joints angles, and they are extracted directly from T6(1:3,4)
    Eigen::VectorXd v=mat*Eigen::Vector4d(0,0,0,1);
    std::vector<float> res;    
    res.push_back(v(0));    res.push_back(v(1));    res.push_back(v(2));

    /* Here we deduce the three angles that express the orientation for the given joints values (a), Roll, Pitch and Yaw. Given T6 we use the rotation matrix part (T6(1:3,1:3))
     * to deduce those angles as follows:
     * Roll = atan2(T6(2,1),T6(1,1))
     * (if both T6(2,1) & T6(1,1) are zeros there is singularity, this means we need to avoid rotation about y axis by +pi/2 or -pi/2), then:
     *  Pitch = atan2(-T6(3,1),cos(Roll)*T6(1,1) + sin(Roll)*T6(2,1)) and
     *  Yaw = atan2(sin(Roll)*T6(1,3) - cos(Roll)*T6(2,3),-sin(Roll)*T6(1,2) + cos(Roll)*T6(2,2)).
     * Refer to [1] for more details.
     * */
    Roll = atan2(mat(1,0),mat(0,0));
    Pitch = atan2(-mat(2,0),cos(Roll)*mat(0,0) + sin(Roll)*mat(1,0));
    Yaw = atan2(sin(Roll)*mat(0,2) - cos(Roll)*mat(1,2),-sin(Roll)*mat(0,1) + cos(Roll)*mat(1,1));

    //complete the pose with orientation part
    res.push_back(Yaw);    res.push_back(Pitch);    res.push_back(Roll);
    return res;
}

//here is the intialization of the inverse kinematic part, where it will set-up the distance vector
void Kinematics::control_inverse_initialize(std::vector<float> target_pos, std::vector<float> initial_joint_values)
{
    //first gets the starting pose according to the starting joint angles
    initial_pos = forward_model(initial_joint_values);
    //then deduce the distance vector using only the position part of the initial_pos vector, and the target position
    distance << target_pos[0] - initial_pos[0], target_pos[1] - initial_pos[1], target_pos[2] - initial_pos[2];
}


/* Here is the inverse kinematic model that produce joints velocities. It takes as input: the target pose (6x1 vector), the starting joints values (to produce the initial pose),
 * and the duration (which will be used to produce the desired linear velocities via rtdot.
 * Refer to [1]  and "Identification and Control of Robots Part II: Control" by Wisama KHALIL [2] for more details.
 * */
std::vector<float> Kinematics::control_inverse(std::vector<float> actual_joint_values, float duration, float current_time){
    //get the jacobian for the current joints angles
    Eigen::MatrixXd jt(jacobian(actual_joint_values));
    //use only the first three rows as the task space is limited to the three cartesian positions only
    Eigen::MatrixXd jt_p = jt.block<3, 6>(0, 0);

    //this will be the output of the function, it is constructed each iteration to ensure its emptyness
    std::vector<float> joints_velocity;

    //construct the twist for current iteration
    Eigen::VectorXd desired_kinematic_twist(3);
    rtdot =(30*pow(current_time,2))/pow(duration,3) - (60*pow(current_time,3))/pow(duration,4) + (30*pow(current_time,4))/pow(duration,5);
    desired_kinematic_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2);

    //do the singular value decomposition which will then help in solving the system jt_p*joint_velocities = desired_twist. Refer to [1] for more details.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jt_p, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd delta_joint(svd.solve(desired_kinematic_twist));

    //fill the joints velocity with corresponding values from delta_joint variable, then return the vector
    for (int i = 0;i < delta_joint.size();i++)
        joints_velocity.push_back(delta_joint(i));

    return joints_velocity;
}

//Guides the arm to desired position by setting joints velocities to proper values each iteration till the duration time is reached
//for integrating the goto_desired_position method with CAFER we decided to split it into two parts: initialization part (init_motion(target)) which will be kept here, then the
//while loop will be moved to controller_node
void Kinematics::init_motion(std::vector<float> desired_position){
    std::vector<float> joints_speeds(7);
    std::vector<float> last_angles;
    duration = 6;
    Real robot;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++){
        joints_speeds[k] = 0.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position,last_angles);
}

/*void Kinematics::goto_desired_position(std::vector<float> desired_position){
    std::vector<float> joints_speeds(7);
    std::vector<float> joints_velocity,last_angles,my_last_angles;
    duration = 6;
    Real robot;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++){
        joints_speeds[k] = 0.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position,last_angles);

    //perform the trajectory in the desired time:

    //first initialize a timer
    boost::timer initial_time_here;*/


    /*
    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(initial_time_here.elapsed() < duration)
    {
        //at each iteration get the current joint angles, as described before
        my_last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < my_last_angles.size(); i++)
            last_angles[i] = M_PI*my_last_angles[i] - initial_joint_values[i];*/
        /*solve for current desired joints speeds, and it appears that the return speeds are actually in RPS (Revolution Per Second) rather than radian/sec, more analysis is to
         * be carried out to see exactly why it gives this, but for the moment it simply means they don't any conversio except for converting them to RPM (Revolution Per Minute)
         * as this what the crustcrawler motors expect as speed command, this is done in the method (set_joint_speeds()) which belongs to robotArm class
        */
        /*
        joints_velocity = control_inverse(last_angles,duration,initial_time_here.elapsed());
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
    }
    //the end effector should be now at the desired position, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}*/

//Guides the real arm to desired joints angles by setting joints velocities to proper values each iteration till the duration time is reached
void Kinematics::goto_desired_joints_angles_velocity_mode(std::vector<float> desired_joints_angles){
    //this will be the velocity command send to the joints
    std::vector<float> joints_velocity(6),angles_distance(6);
    std::vector<float> joints_speeds(7);

    //instantiate a robot class to use for controlling the arm
    Real robot;

    /* get the distance to be covered by each joint by reading the current position and then subtract from it the desired joints angles
     * */
    /***************************check the distances may be they aren't what they supposed to be ********/
    std::vector<float> starting_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    for (int i = 0; i < 6; i++)
        angles_distance[i] = desired_joints_angles[i] - M_PI*starting_angles[i];

    //set the time, in seconds, to complete the trajectory in joints space
    float duration = 8;

    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++){
        joints_speeds[k] = 0.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //perform the trajectory in the desired time:

    //instantiate the interpolation parameter
    float rtdot;
    float time_passed = 0;
    // first initialize a timer:
    boost::timer time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(time_here.elapsed() < duration)
    {
        //at each iteration get the current value for the interpolation parameter
        rtdot =(30*pow(time_here.elapsed(),2))/pow(duration,3) - (60*pow(time_here.elapsed(),3))/pow(duration,4) + (30*pow(time_here.elapsed(),4))/pow(duration,5);

        //set joints velocities, by multiplying the rtdot interpolation parameter by the distance of joint (j)
        for (int j = 0;j < 6;j++)
            joints_velocity[j] = rtdot*angles_distance[j];
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
        //std::cout << "loop duration is: " << time_here.elapsed() - time_passed << std::endl;
        std::cout << "speed of important joint is: " << joints_velocity[2]*60 << std::endl;
        time_passed = time_here.elapsed();
    }
    std::cout << "final loop duration is: " << time_here.elapsed() << std::endl;
    starting_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    for (int i = 0; i < 6; i++)
       std::cout << "finishing angle for joint: " << i << " is: " << M_PI*starting_angles[i] << std::endl;
    //the joints should be now at their desired angles, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}

void Kinematics::goto_desired_joints_angles_position_mode(std::vector<float> desired_joints_angles){
    //this will be the velocity command send to the joints
    std::vector<float> joints_speeds(7);

    //instantiate a robot class to use for controlling the arm
    Real robot;
    for (int i = 0; i < 6; i++)
        desired_joints_angles[i] = desired_joints_angles[i]/M_PI;

    robot.getArm().mode_position(reduced_actuator_id);
    //set motors speeds to a desired speeds that will be used to reach the value of each joint
    for (int k = 0; k < reduced_actuator_id.size(); k++){
        joints_speeds[k] = 70.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds,reduced_actuator_id);

    robot.getArm().set_joint_values(desired_joints_angles,reduced_actuator_id);
    std::vector<float> finishing_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    for (int i = 0; i < 6; i++)
       std::cout << "finishing angle for joint: " << i << " is: " << M_PI*finishing_angles[i] << std::endl;
    //the joints should be now at their desired angles, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}
