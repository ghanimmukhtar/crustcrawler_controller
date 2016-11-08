#include "kinematics.hpp"
#include <fstream>
#include <iostream>
#include <typeinfo>

#define EPSILON 0.01

using namespace robot;

/* Global variables:
 * these variables are defined here because they are global besides i had trouble initializing them in the kinematic header file, may be their placement will be changed later
*/
//This defines the euclidean distance between initial_pos and target position for x, y and z coordinates
Eigen::VectorXd distance(3),joints_distance(6);
std::vector<double> start_position, current_cart_position;
std::ofstream desired_velocity,feedback_velocity, desired_pose,fdb_position;
std::vector<std::vector<double>> my_orientations; //to keep track of the orientation of each executed motion

/*This defines the angles of each joint upon which the geometric parameters table was built, this means that these angles are the zero position and each joint angle should be
 * measured with reference to them
 * */
std::vector<double> initial_joint_values = {0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0};

//These are the joints which actually matter when we want to use the FK or IK, we exclude the two motors for the gripper
std::vector<unsigned char> reduced_actuator_id = {1, 2, 3, 4, 5, 6, 7};

//This is a function that takes as inputs: a character that name an axis (x, y or z), and a double that represents an angle of rotation about that axis. It returns the corresponding rotation matrix (3x3)
Eigen::Matrix3d Kinematics::Rot(char axis, double angle) const{
    Eigen::Matrix3d RotX,RotY,RotZ,empty_Rot;
    if (axis == 'x'){
        RotX << 1,          0,           0,
                0, cos(angle), -sin(angle),
                0, sin(angle),  cos(angle);
        return RotX;
    }
    else if (axis == 'y'){
        RotY << cos(angle),  0, sin(angle),
                         0,  1,          0,
               -sin(angle),  0, cos(angle);
        return RotY;
    }
    else if (axis == 'z'){
        RotZ << cos(angle), -sin(angle), 0,
                sin(angle),  cos(angle), 0,
                         0,           0, 1;
        return RotZ;
    }
    else{
        std::cout << "enter valid coordinate: X, Y or Z -------------------------------------------------" << std::endl;
        return empty_Rot;
    }

}

//geometric parameters according to the choosing initial configurations defined by initial_joint_values
Eigen::MatrixXd Kinematics::kk_mat(const std::vector<double>& a) const
{
    //robot description (dimensions of each link and joint)
    double body = Params::body_height,
            p1 = Params::p1_height,
            p2 = Params::p2_height,
            p3 = Params::p3_height,
            p4 = Params::p4_height,
            p5 = Params::p5_height,
            p6 = Params::p6_height,
            gripper_height = Params::gripper_height;
     //double body = 0.05, p1 = 0.056, p2 = 0.23, p3 = 0.155, p4 = 0.08, p5 = 0.07, p6 = 0.021, gripper_height = 0.171;

    /* This is the geometric representation of the robot according to Khalil-Kleinfinger notation, the additional colomn is to represent the type of the joint it is 0 for
     * revolute, 1 for translational and 2 for fixed joint, it will be useful later for the dynamic model if needed, for more details refer to : "Modeling and Control of
     * Manipulators Part I: Geometric and Kinematic Models" by Wisama KHALIL [1]
     * */
    Eigen::MatrixXd kk(6, 5);
    //---|---sigma---|---alpha_j---|---d_j---|---q_j--------|---r_j---|
    kk <<      0,         0,          0,       a[0],       body+p1,
               0,       M_PI/2,       0,       a[1],           0,
               0,         0,         p2,       a[2],           0,
               0,      -M_PI/2,       0,       a[3],           0.21,  //p3+p4,
               0,       M_PI/2,       0,       a[4],           0,
               0,      -M_PI/2,       0,       a[5],  gripper_height+p5+p6;
    return kk;
}


/* So kk is a row in geometric parameter table defined in kk_mat(), so for row kk(j) this function will return the homogeneous transfromation between frame (j-1) and (j).
 * Actually this transformation matrix (Tj) is formulated as follows: Tj = Rot(x,alpha_j)*Trans(x,d_j)*Rot(z,q_j)*Trans(z,r_j). For more details refer to [1]
 * */
Eigen::Matrix4d Kinematics::trmat_kk(const Eigen::VectorXd& kk) const
{
    Eigen::Matrix4d submat;
    submat << cos(kk(3)), -sin(kk(3)), 0, kk(2),
            cos(kk(1)) * sin(kk(3)), cos(kk(1)) * cos(kk(3)), -sin(kk(1)), -kk(4) * sin(kk(1)),
            sin(kk(1)) * sin(kk(3)), sin(kk(1)) * cos(kk(3)), cos(kk(1)), kk(4) * cos(kk(1)),
            0, 0, 0, 1;
    return submat;
}

//gives all intermediate transformation matrices that will be used to generate the jacobian later
std::vector<Eigen::Matrix4d> Kinematics::sub_trmat(const std::vector<double>& a) const
{
    std::vector<Eigen::Matrix4d> mats;
    Eigen::MatrixXd kk = kk_mat(a);
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity(4, 4);
    for (size_t i = 0; i < kk.rows(); i++) {
        //transformation between frame i and i-1, when i is 0 it gives the transformation between frame of first joint (turntable joint) and the world frame (fixed frame)
        mat = mat * trmat_kk(kk.row(i));
        mats.push_back(mat);
    }
    return mats;
}

//the kinematic jacobian is calculated here,
Eigen::MatrixXd Kinematics::jacobian(const std::vector<double>& a) const
{
    std::vector<Eigen::Matrix4d> mats = sub_trmat(a);
    Eigen::MatrixXd jacob(6, 6);
    for (int i = 0; i < mats.size(); i++) {
        /*the formulation of column (i) is as follows: J(1:3,i) = [a_i x L_i_n], and J(4:6,i) = [a_i], with (a_i) being the third column of the transformation matrix for frame (i)
        (i.e. the vector that express axis of rotation for joint i) and (L_i_n) being the distance vector between frame (i) and the frame of end effector (frame 6 in this particular
        case), this distance vector is calculated by subtracting from the fourth column of the last transformation matrix T6(1:3,4) the one of Ti(1:3,4).
        Refer to [1] for more details.*/
        jacob.col(i).head(3) = ((Eigen::Vector3d) mats[i].col(2).head(3)).cross(
                (Eigen::Vector3d) mats[5].col(3).head(3) - (Eigen::Vector3d) mats[i].col(3).head(3));
        jacob.col(i).tail(3) = mats[i].col(2).head(3);
    }
    return jacob;
}


void Kinematics::pseudo_inverse(const Eigen::MatrixXd& mat, Eigen::MatrixXd& invMat)
{
    //do the singular value decomposition which will then help in solving the system jt_p*joint_velocities = desired_twist. Refer to [1] for more details.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

    if (!svd.computeV() || !svd.computeU()) {
        std::cout << "impossible to compute svd" << std::endl;
        exit(1);
    }

    Eigen::MatrixXd invSigma(3, 3);
    for (int i = 0; i < invSigma.rows(); i++) {
        for (int j = 0; j < invSigma.cols(); j++) {
            if (i == j) {
                assert(svd.singularValues()(i) != 0);
                invSigma(i, j) = 1 / svd.singularValues()(i);
            }
            else { invSigma(i, j) = 0; }
        }
    }

    invMat = svd.matrixV() * invSigma * svd.matrixU().transpose();
}

/* The output of the forward model is the three position Cartesian coordinates and three angles (Roll, Pitch, Yaw) with the following order of rotations
* (Rot = Rot(z,Roll)*Rot(y,Pitch)*Rot(x,Yaw)), which gives the following 3x3 rotation matrix:
* [ cos(Roll)*cos(Pitch),  cos(Roll)*sin(Pitch)*sin(Yaw) - sin(Roll)*cos(Yaw), cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw);
*   sin(Roll)*cos(Pitch),  sin(Roll)*sin(Pitch)*sin(Yaw) + cos(Roll)*cos(Yaw), sin(Roll)*sin(Pitch)*cos(Yaw) - cos(Roll)*sin(Yaw);
*            -sin(Pitch),                                 cos(Pitch)*sin(Yaw),                                 cos(Pitch)*cos(Yaw)]
* Refer to [1] for more details.
* */
std::vector<double> Kinematics::forward_model(const std::vector<double>& a) const
{
    //first construct the geometric parameters matrix as described in Khalil-Kleinfinger notation
    Eigen::MatrixXd kk = kk_mat(a);
    //this will the transformation matrix between each joint frame and subsequent one
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity(4, 4);
    //orientation angles about X, Y and Z
    double Roll;
    double Pitch;
    double Yaw;

    /* This should give the transformation matrix from the frame 0 (F0) to frame 6 (F6) which is at the end of the last actuator (let's denote it T0T6), so to get the pose of
     * the end effector in the absolute world frame two transformations need to be accounted for: first the transformation from F0 to the world absolute frame (let's call it Fw),
     * we can denote it as TwT0, second the transformation from the frame attached to  the end effector (let's call it Fe) to F6 (T6Te). For the complete description of all
     * frames look at the documentation of the code. Refer to [1] for more details.
     * */
    for (size_t i = 0; i < kk.rows(); i++) {
        mat = mat * trmat_kk(kk.row(i));
    }

    //The variable v contains the Cartesian position coordinates for the given joints angles, and they are extracted directly from T6(1:3,4)
    Eigen::VectorXd v = mat * Eigen::Vector4d(0, 0, 0, 1);
    std::vector<double> res;
    res.push_back(v(0));
    res.push_back(v(1));
    res.push_back(v(2));

    /* Here we deduce the three angles that express the orientation for the given joints values (a), Roll, Pitch and Yaw. Given T6 we use the rotation matrix part (T6(1:3,1:3))
     * to deduce those angles as follows:
     * Roll = atan2(T6(2,1),T6(1,1))
     * (if both T6(2,1) & T6(1,1) are zeros there is singularity, this means we need to avoid rotation about y axis by +pi/2 or -pi/2), then:
     *  Pitch = atan2(-T6(3,1),cos(Roll)*T6(1,1) + sin(Roll)*T6(2,1)) and
     *  Yaw = atan2(sin(Roll)*T6(1,3) - cos(Roll)*T6(2,3),-sin(Roll)*T6(1,2) + cos(Roll)*T6(2,2)).
     * Refer to [1] for more details.
     * */
    Pitch = asin(mat(0,2));
    if (Pitch == M_PI/2)
    {
        Yaw = 0;
        Roll = atan2(mat(1,0),-mat(2,0));
    }
    else if (Pitch == -M_PI/2)
    {
        Yaw = 0;
        Roll = -atan2(mat(1,0),mat(2,0));
    }
    else
    {
        Yaw = atan2(-mat(0,1)/cos(Pitch),mat(0,0)/cos(Pitch));
        Roll = atan2(-mat(1,2)/cos(Pitch),mat(2,2)/cos(Pitch));
    }
    /*
    Roll = atan2(mat(1, 0), mat(0, 0));
    Pitch = atan2(-mat(2, 0), cos(Roll) * mat(0, 0) + sin(Roll) * mat(1, 0));
    Yaw = atan2(sin(Roll) * mat(0, 2) - cos(Roll) * mat(1, 2), -sin(Roll) * mat(0, 1) + cos(Roll) * mat(1, 1));
    */
    //complete the pose with orientation part
    res.push_back(Roll);
    res.push_back(Pitch);
    res.push_back(Yaw);
    return res;
}

//here is the intialization of the inverse kinematic part, where it will set-up the distance vector
void Kinematics::control_inverse_initialize(std::vector<double> target_pos, std::vector<double> initial_joint_values)
{
    //first gets the starting pose according to the starting joint angles
    initial_pos = forward_model(initial_joint_values);
    //then deduce the distance vector using only the position part of the initial_pos vector, and the target position
    distance << target_pos[0] - initial_pos[0], target_pos[1] - initial_pos[1], target_pos[2] - initial_pos[2];

    //orientation part
    Eigen::Matrix3d Rfinal;
    Eigen::Matrix3d Rinitial = Rot('x',initial_pos[3])*Rot('y',initial_pos[4])*Rot('z',initial_pos[5]);
    if(target_pos[3] == 0.0 && target_pos[4] == 0.0 && target_pos[5] == 0.0)
        Rfinal = Rinitial;
    else
        Rfinal = Rot('z',target_pos[5])*Rot('y',target_pos[4])*Rot('x',target_pos[3]);
    //std::cout << "initial orientation is: " << std::endl << Rinitial << std::endl;
    //std::cout << "final orientation should be: " << std::endl << Rfinal << std::endl;

    //then deduce the difference between the intial and final points of a section in oreintation:
    Eigen::Matrix3d RotuAlpha = Rfinal*Rinitial.transpose();

    //then we extract the angle alpha:
    double Calpha = 0.5*(RotuAlpha(0,0)+RotuAlpha(1,1)+RotuAlpha(2,2)-1);
    double Salpha = 0.5*sqrt(pow(RotuAlpha(1,2) - RotuAlpha(2,1),2) + pow(RotuAlpha(2,0) - RotuAlpha(0,2),2) + pow(RotuAlpha(0,1) - RotuAlpha(1,0),2));
    my_alpha = atan2(Salpha,Calpha);

    //now that we have alpha let's deduce the vector u and skew symmetric which will be used to evaluate the evolution of the orientation with times
    if (my_alpha == 0){
        u = Eigen::VectorXd::Zero(3);
    }
    else{
        u << (RotuAlpha(2,1)-RotuAlpha(1,2))/(2*Salpha),
            (RotuAlpha(0,2)-RotuAlpha(2,0))/(2*Salpha),
            (RotuAlpha(1,0)-RotuAlpha(0,1))/(2*Salpha);
    }
    //std::cout << "vector u is: " << std::endl << u << std::endl;

    //std::cout << "distance norm" << distance.norm() << std::endl;
    duration = std::max(7.5*u.norm()/max_speed,7.5*distance.norm()/max_speed);
    //duration = 7.5*distance.norm()/max_speed;
    std::cout << "duration is: " << duration << std::endl;
    std::cout << "Distance to be covered: \n" << distance << std::endl;
}


/* Here is the inverse kinematic model that produce joints velocities. It takes as input: the target pose (6x1 vector), the starting joints values (to produce the initial pose),
 * and the duration (which will be used to produce the desired linear velocities via rtdot.
 * Refer to [1]  and "Identification and Control of Robots Part II: Control" by Wisama KHALIL [2] for more details.
 * */
std::vector<double> Kinematics::control_inverse(std::vector<double> actual_joint_values, double duration,
                                                double current_time)
{

    //get the jacobian for the current joints angles
    Eigen::MatrixXd jt(jacobian(actual_joint_values));

    //use only the first three rows as the task space is limited to the three cartesian positions only
    //Eigen::MatrixXd jt_p = jt.block<3, 6>(0, 0);
    //if(fabs((jt_p * jt_p.transpose()).determinant()) < 0.00008){
     //   std::cout << "jacobian determinant is less than 0.01: " << std::endl;
     //   std::cout << (jt_p * jt_p.transpose()).determinant() << std::endl;
    //}

    //Part for joints limits avoidance, it doesn't work when we include the orientation part because the jacobian is not redundant anymore
    /*
    Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd Z(6);
    Eigen::VectorXd joint_val(6);
    joint_val << actual_joint_values[0]
              , actual_joint_values[1] + initial_joint_values[1]
              , actual_joint_values[2] + initial_joint_values[2]
              , actual_joint_values[3]
              , actual_joint_values[4]
              , actual_joint_values[5];
    double a = -.05;

    double delta_j_limit = M_PI;
    Z = 2 * a * joint_val / (delta_j_limit * delta_j_limit);*/

    //std::cout << " ******************** i am here ************************ " << std::endl;
    //this will be the output of the function, it is constructed each iteration to ensure its emptyness
    std::vector<double> joints_velocity;

    //construct the twist for current iteration
    Eigen::VectorXd desired_kinematic_twist(6);
    rtdot = (30 * pow(current_time, 2)) / pow(duration, 3) - (60 * pow(current_time, 3)) / pow(duration, 4) +
            (30 * pow(current_time, 4)) / pow(duration, 5);
    desired_kinematic_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2), rtdot * my_alpha * u(0), rtdot * my_alpha * u(1), rtdot * my_alpha * u(2);
    /*start_position[0] = start_position[0] + desired_kinematic_twist(0)*0.03;
    start_position[1] = start_position[1] + desired_kinematic_twist(1)*0.03;
    start_position[2] = start_position[2] + desired_kinematic_twist(2)*0.03;*/

    //Eigen::MatrixXd invJt_p(6, 3);
    //pseudo_inverse(jt_p, invJt_p);
    if(fabs(jt.determinant()) < 1e-6){
        std::cout << jt << std::endl
                     << "*********************************************" << std::endl;
        std::cout << " SINGULARITY !!" << std::endl;
        //return zero_velocities
        for (int i = 0;i<actual_joint_values.size();i++)
            joints_velocity.push_back(0);
        return joints_velocity;
    }
    //std::cout << jt.determinant() << std::endl
      //           << "*********************************************" << std::endl;
    //do the singular value decomposition which will then help in solving the system jt_p*joint_velocities = desired_twist. Refer to [1] for more details.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd delta_joint(svd.solve(desired_kinematic_twist));

    //fill the joints velocity with corresponding values from delta_joint variable, then return the vector
    for (int i = 0;i < delta_joint.size();i++)
        joints_velocity.push_back(delta_joint(i));
    //Eigen::VectorXd delta_joint(invJt_p*desired_kinematic_twist + (Id - invJt_p*jt_p)*Z);
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd(jt_p, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Eigen::VectorXd delta_joint(svd.solve(desired_kinematic_twist));
    return joints_velocity;
}

/*Guides the arm to desired position by setting joints velocities to proper values each iteration till the duration time is reached
  for integrating the goto_desired_position method with CAFER we decided to split it into two parts: initialization part (init_motion(target)) which will be kept here, then the
  while loop will be moved to controller_node*/
void Kinematics::init_motion(std::vector<double> desired_position)
{
    std::vector<double> joints_speeds(7);
    std::vector<double> last_angles;
//    duration = d;
    Real robot;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++) {
        joints_speeds[k] = 0.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++) {
        last_angles[i] = M_PI * last_angles[i] - initial_joint_values[i];
    }

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position, last_angles);
}

/*Guides the arm to desired position by setting joints positions to proper values each iteration till the duration time is reached
  for integrating the goto_desired_position_in_position mode method with CAFER we decided to split it into two parts: initialization part (init_motion_position(target)) which will be kept here, then the
  while loop will be moved to controller_node
void Kinematics::init_motion_position(std::vector<double> desired_position)
{
    std::vector<double> joints_speeds(7);
    std::vector<double> last_angles;
    Real robot;
    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++) {
        joints_speeds[k] = max_angular_speed;
    }
    robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++) {
        last_angles[i] = M_PI * last_angles[i] - initial_joint_values[i];
    }

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position, last_angles);
}*/

void Kinematics::releave(){
    std::cout << "i am stressed ******************************" << std::endl;
    std::vector<double> target_position(3),last_angles,current_position,joints_velocity;
    Real robot;
    double prim_distance = -0.05;
    //do the retract motion and then up
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];
    current_position = forward_model(last_angles);

    //get the diagonal line from the origin to the end effector
    double angle = atan2(current_position[1],current_position[0]);

    //set the target position which will have the same z coordinate but x and y will change according to the direction defined above
    target_position[0] = current_position[0] + prim_distance*cos(angle);
    target_position[1] = current_position[1] + prim_distance*sin(angle);
    target_position[2] = current_position[2];

    //perform the motion using the method goto_desired_position
    //goto_desired_position(target_position);
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(target_position,last_angles);

    //avariable to read joints loads and show them
    //std::vector <double> joints_loads;
    //perform the trajectory in the desired time:

    //first initialize a timer
    boost::timer initial_time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(initial_time_here.elapsed() < duration)
    {
        //at each iteration get the current joint angles, as described before
        last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < last_angles.size(); i++)
            last_angles[i] = M_PI*last_angles[i] - initial_joint_values[i];
        //std::cout << "************** i am here doing releave motion ****************" << std::endl;
        /*solve for current desired joints speeds, and it appears that the return speeds are actually in RPS (Revolution Per Second) rather than radian/sec, more analysis is to
         * be carried out to see exactly why it gives this, but for the moment it simply means they don't any conversio except for converting them to RPM (Revolution Per Minute)
         * as this what the crustcrawler motors expect as speed command, this is done in the method (set_joint_speeds()) which belongs to robotArm class
        */

        joints_velocity = control_inverse(last_angles,duration,initial_time_here.elapsed());
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
        /*joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++)
            std::cout << joints_loads[i] << std::endl;
        std::cout << "*****************************************" << std::endl;*/
    }
    robot.getArm().set_speeds_to_zero(reduced_actuator_id);
    robot.getArm().close_usb_controllers();
}

void Kinematics::goto_desired_position(std::vector<double> desired_position){
    std::vector<double> joints_speeds(7);
    std::vector<double> joints_velocity,last_angles,my_last_angles;
//    duration = d;

    Real robot;

        //set motors speed to zero so that they start from static situation
        for (int k = 0; k < reduced_actuator_id.size(); k++) {
            joints_speeds[k] = 0.0;
        }
        robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);

        //prepare motors to receive commands in velocity (change them to wheel mode)
        robot.getArm().mode_speed(reduced_actuator_id);

        //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
        //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
        last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < last_angles.size(); i++) {
            last_angles[i] = M_PI * last_angles[i] - initial_joint_values[i];
        }

    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position, last_angles);

    //boolean variable to know when the arm is overlaoded
    bool stressed = false;
    //avariable to read joints loads and show them
    std::vector <double> joints_loads;

    //first initialize a timer
    boost::timer initial_time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(initial_time_here.elapsed() < duration)
    {
        joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++){
            //std::cout << joints_loads[i] << std::endl;
            if (joints_loads[i] > max_load){
                stressed = true;
                break;
            }
        }
        if (stressed)
            break;
        //at each iteration get the current joint angles, as described before
        my_last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < my_last_angles.size(); i++)
            last_angles[i] = M_PI*my_last_angles[i] - initial_joint_values[i];
        /*solve for current desired joints speeds, and it appears that the return speeds are actually in RPS (Revolution Per Second) rather than radian/sec, more analysis is to
         * be carried out to see exactly why it gives this, but for the moment it simply means they don't any conversio except for converting them to RPM (Revolution Per Minute)
         * as this what the crustcrawler motors expect as speed command, this is done in the method (set_joint_speeds()) which belongs to robotArm class
        */

        joints_velocity = control_inverse(last_angles,duration,initial_time_here.elapsed());
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
        /*joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++)
            std::cout << joints_loads[i] << std::endl;
        std::cout << "*****************************************" << std::endl;*/
    }
    robot.getArm().set_speeds_to_zero(reduced_actuator_id);

    //if the arm collides with fixed obstacles stop the motion and releave the arm
    if (stressed){
        releave();
        stressed = false;
    }

    //the end effector should be now at the desired position, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}

void Kinematics::goto_desired_position_without_stress(std::vector<double> desired_position){
    std::vector<double> joints_speeds(7);
    std::vector<double> joints_velocity,last_angles,my_last_angles,vel_fdb;
    desired_velocity.open("my_des_vel.csv"); feedback_velocity.open("my_fdb_vel.csv");
    desired_pose.open("desired_position.csv"); fdb_position.open("feedback_position.csv");
//    duration = d;

    Real robot;

        //set motors speed to zero so that they start from static situation
        for (int k = 0; k < reduced_actuator_id.size(); k++) {
            joints_speeds[k] = 0.0;
        }
        robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);

        //prepare motors to receive commands in velocity (change them to wheel mode)
        robot.getArm().mode_speed(reduced_actuator_id);

        //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
        //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
        last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < last_angles.size(); i++) {
            last_angles[i] = M_PI * last_angles[i] - initial_joint_values[i];
        }
    start_position = forward_model(last_angles);
    current_cart_position = start_position;
    desired_pose << start_position[0] << "," << start_position[1] << "," << start_position[2] << "\n";
    fdb_position << current_cart_position[0] << ","<< current_cart_position[1] << ","<< current_cart_position[2] << "\n";
    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position, last_angles);

    //double my_time = 0.0, iteration_duration = 0.0;
    //first initialize a timer
    boost::timer initial_time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(initial_time_here.elapsed() < duration)
    {

        //my_time = initial_time_here.elapsed();
        //at each iteration get the current joint angles, as described before
        my_last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
        for (int i = 0; i < my_last_angles.size(); i++)
            last_angles[i] = M_PI*my_last_angles[i] - initial_joint_values[i];
        current_cart_position = forward_model(last_angles);
        /*solve for current desired joints speeds, and it appears that the return speeds are actually in RPS (Revolution Per Second) rather than radian/sec, more analysis is to
         * be carried out to see exactly why it gives this, but for the moment it simply means they don't any conversio except for converting them to RPM (Revolution Per Minute)
         * as this what the crustcrawler motors expect as speed command, this is done in the method (set_joint_speeds()) which belongs to robotArm class
        */
        joints_velocity = control_inverse(last_angles,duration,initial_time_here.elapsed());
        //joints_velocity[2] = 3*joints_velocity[2];
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity,reduced_actuator_id);
        //vel_fdb = robot.getArm().get_joint_speeds(reduced_actuator_id);
        //desired_velocity << joints_velocity[0] << "," << joints_velocity[1] << "," << joints_velocity[2] << "," << joints_velocity[3] << "," << joints_velocity[4] << "," <<
                                                   //joints_velocity[5] << "," << "\n";
        //feedback_velocity << vel_fdb[0] << "," << vel_fdb[1] << "," << vel_fdb[2] << "," << vel_fdb[3] << "," << vel_fdb[4] << "," << vel_fdb[5] << "," << "\n";
        desired_pose << start_position[0] << "," << start_position[1] << "," << start_position[2] << "\n";
        fdb_position << current_cart_position[0] << ","<< current_cart_position[1] << ","<< current_cart_position[2] << "\n";
        /*joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++)
            std::cout << joints_loads[i] << std::endl;
        std::cout << "*****************************************" << std::endl;*/
        //iteration_duration = initial_time_here.elapsed() - my_time;
        //std::cout << "iteration time is: " << iteration_duration << std::endl;
        //std::cout << "*****************************************" << std::endl;
    }
    robot.getArm().set_speeds_to_zero(reduced_actuator_id);
    //desired_velocity.close();
    //feedback_velocity.close();
    desired_pose.close();
    fdb_position.close();
    //the end effector should be now at the desired position, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}

bool Kinematics::goto_desired_position_in_position_mode(std::vector<double> desired_position,std::vector<double>& joints_values){
    std::vector<double> joints_f_values;
    //if orientation isn't given just move with end effector pointing ahead in the direction of final position
    if(desired_position.size() < 6) {
        double angle = atan2(desired_position[1],desired_position[0]);
        desired_position.push_back(0); desired_position.push_back(2.3); desired_position.push_back(angle);
    }
    joints_f_values = return_final_joints_values(desired_position,joints_values);
    for(int i=0;i<joints_f_values.size();i++)
        std::cout << "original theoritical finishing angle for joint: " << i << " is: " << joints_f_values[i] << std::endl;
    for(int i=0;i<joints_f_values.size();i++)
        joints_f_values[i] = fmod(joints_f_values[i], M_PI);
    for(int i=0;i<joints_f_values.size();i++) {
        if(joints_f_values[i] <= -M_PI)
            joints_f_values[i] = joints_f_values[i] + M_PI;
        else if (joints_f_values[i] > M_PI)
            joints_f_values[i] = joints_f_values[i] - M_PI;
    }
    //bool output = false;
    bool output = goto_desired_joints_angles_position_mode(joints_f_values);
    for(int i=0;i<joints_f_values.size();i++)
        std::cout << "theoritical finishing angle for joint: " << i << " is: " << joints_f_values[i] << std::endl;
    return output;
}

//simple method to limit an angle between Pi and -Pi
double Kinematics::angle_pi(double angle){
    if (angle > 2*M_PI)
        angle = fmod(angle, 2*M_PI);
    if(angle <= -M_PI)
        angle = 2*M_PI + angle;
    else if (angle > M_PI)
        angle = angle - 2*M_PI;
    return angle;
}

//a complete method that generates all possible joints solutions, for all orientations regarding a given position (X, Y, Z). For orientation around z it is fixed to have only meaningful solutions, but we can add it if needed
std::vector<std::vector<double>> Kinematics::IG_complete(std::vector<double> desired_position){
                                 std::vector<std::vector<double>> all_orientations, temperarily_container;
                                 double Roll = 0, Pitch = M_PI/2.0, Yaw = atan2(desired_position[1], desired_position[0]), step = 0.6;
                                 //while(Yaw < 2*M_PI){
                                 //Pitch = M_PI/2.0;
                                 while(Pitch < 2*M_PI){
                                 Roll = 0;
                                 while(Roll < 2*M_PI){
                                 temperarily_container.clear();
                                 std::vector<double> desired_pose = {desired_position[0], desired_position[1], desired_position[2], Roll, Pitch, Yaw};
                                 temperarily_container = All_ig_solutions(desired_pose);
                                 //for(int i = 0; i < temperarily_container.size(); ++i)
                                 if(!temperarily_container.empty()){
                                    all_orientations.push_back(temperarily_container[0]);
                                    my_orientations.push_back({Roll, Pitch, Yaw});
                                 }
                                 //std::cout << "solving for orientation: " << Yaw << " , " << Pitch << " , " << Roll << std::endl;
                                 Roll += step;
                                 }
                                 //std::cout << "*****************************************" << std::endl;
                                 Pitch += step;
                                 }
                                 //Yaw += step;
                                 //}
                                 for(int i = 0; i < all_orientations.size(); ++i){
                                     for(int j = 0; j < all_orientations.size(); ++j){
                                         if(i != j){
                                             if(all_orientations[i] == all_orientations[j]){
                                                 std::cout << "two solutions are identical, solution: " << i << " and solution: " << j << std::endl;
                                                 all_orientations.erase(all_orientations.begin() + j);
                                             }
                                         }
                                     }
                                 }
                                 std::cout << "final solution size with all orientations is: " << all_orientations.size() << std::endl;
                                 return all_orientations;
}

//this method returns solutions that respect joints limits among the four "possible" solutions for the crustcrawler for a given pose (position and orientation)
std::vector<std::vector<double>> Kinematics::All_ig_solutions(std::vector<double> desired_pose){
                                 //this will be the command send to the joints
                                 std::vector<std::vector<double>> final_joints_angles;
                                 std::vector<double> joints_angles;
                                 double ang_dist = 0.4;
                                 Eigen::VectorXd min_joint_limit(6), max_joint_limit(6);
                                 min_joint_limit << -M_PI/2 - ang_dist, -M_PI/2 - ang_dist, -M_PI/2 - ang_dist, -M_PI/2 - ang_dist, -M_PI/2 - ang_dist, -M_PI/2 - ang_dist;
                                 max_joint_limit <<  M_PI/2 + ang_dist,  M_PI/2 + ang_dist,  M_PI/2 + ang_dist,  M_PI/2 + ang_dist,  M_PI/2 + ang_dist,  M_PI/2 + ang_dist;
                                 double q1, q2, q3, q4, q5, q6;
                                 double body = robot::Params::body_height, p1 = robot::Params::p1_height, p5 = robot::Params::p5_height, p6 = robot::Params::p6_height, gripper_height = robot::Params::gripper_height;
                                 Eigen::Matrix4d TE_6, TF_E, TT;
                                 //transformation matrix between End Effector's frame and last joint's frame (i.e. 6th joint)
                                 TE_6 << 1, 0, 0,              0,
                                         0, 1, 0,              0,
                                         0, 0, 1, -(gripper_height+p5+p6),
                                         0, 0, 0,              1;
                                 Eigen::Matrix3d Rfinal = Rot('z', desired_pose[5]) * Rot('y', desired_pose[4]) * Rot('x', desired_pose[3]);
                                 TF_E << Rfinal(0,0), Rfinal(0,1), Rfinal(0,2), desired_pose[0],
                                         Rfinal(1,0), Rfinal(1,1), Rfinal(1,2), desired_pose[1],
                                         Rfinal(2,0), Rfinal(2,1), Rfinal(2,2), desired_pose[2],
                                                   0,           0,           0,               1;
                                 //std::cout << "desired final orientation is: \n" << Rfinal << std::endl;
                                 //std::cout << "*******************************************" << std::endl;
                                 TT = TF_E*TE_6;
                                 //find joints angles
                                 double Px = TT(0,3), Py = TT(1,3), Pz = TT(2,3), D3 = robot::Params::p2_height, RL4 = 0.21;
                                 for(int i = 0; i < 4; ++i){
                                     joints_angles.clear();
                                     //q1
                                     if(i == 0 || i == 1)
                                         q1 = angle_pi(atan2(Py, Px));
                                     else
                                         q1 = angle_pi(atan2(Py, Px) + M_PI);
                                     joints_angles.push_back(q1);
                                     double X = 2*(body + p1 - Pz)*D3;
                                     double B1 = Px*cos(q1) + Py*sin(q1);
                                     double Y = -2*B1*D3;
                                     double Z = pow(RL4, 2) - pow(D3, 2) - pow((body + p1 - Pz), 2) - pow(B1, 2);
                                     double c2 = (Y*Z + X*sqrt(pow(X,2) + pow(Y,2) - pow(Z,2)))/(pow(X,2) + pow(Y,2));
                                     double s2 = (X*Z - Y*sqrt(pow(X,2) + pow(Y,2) - pow(Z,2)))/(pow(X,2) + pow(Y,2));
                                     q2 = angle_pi(atan2(s2,c2) + initial_joint_values[1]);
                                     //std::cout << "square root value is: " << pow(X,2) + pow(Y,2) - pow(Z,2) << std::endl;
                                     joints_angles.push_back(q2);
                                     q2 = atan2(s2,c2);
                                     //std::cout << "initial solution for q2 is: " << q2 << std::endl;
                                     //q3
                                     double s3 = ((body + p1 - Pz)*s2 - B1*c2 + D3)/RL4;
                                     double c3 = (-B1*s2 + (Pz - (body + p1))*c2)/RL4;
                                     q3 = angle_pi(atan2(s3,c3) + initial_joint_values[2]);
                                     joints_angles.push_back(q3);
                                     q3 = atan2(s3,c3);
                                     //std::cout << "initial solution for q3 is: " << q3 << std::endl;
                                     //orientation part
                                     double Fx = cos(q2 + q3)*(cos(q1)*Rfinal(0,0) + sin(q1)*Rfinal(1,0)) + sin(q2 + q3)*Rfinal(2,0),
                                            Fy = -sin(q2 + q3)*(cos(q1)*Rfinal(0,0) + sin(q1)*Rfinal(1,0)) + cos(q2 + q3)*Rfinal(2,0),
                                            Fz = sin(q1)*Rfinal(0,0) - cos(q1)*Rfinal(1,0);
                                     double Gx = cos(q2 + q3)*(cos(q1)*Rfinal(0,1) + sin(q1)*Rfinal(1,1)) + sin(q2 + q3)*Rfinal(2,1),
                                            Gy = -sin(q2 + q3)*(cos(q1)*Rfinal(0,1) + sin(q1)*Rfinal(1,1)) + cos(q2 + q3)*Rfinal(2,1),
                                            Gz = sin(q1)*Rfinal(0,1) - cos(q1)*Rfinal(1,1);
                                     double Hx = cos(q2 + q3)*(cos(q1)*Rfinal(0,2) + sin(q1)*Rfinal(1,2)) + sin(q2 + q3)*Rfinal(2,2),
                                            Hy = -sin(q2 + q3)*(cos(q1)*Rfinal(0,2) + sin(q1)*Rfinal(1,2)) + cos(q2 + q3)*Rfinal(2,2),
                                            Hz = sin(q1)*Rfinal(0,2) - cos(q1)*Rfinal(1,2);

                                     //q4
                                     if(i == 0 || i == 2)
                                         q4 = angle_pi(atan2(Hz, -Hx));
                                     else
                                         q4 = angle_pi(atan2(Hz, -Hx) + M_PI);
                                     joints_angles.push_back(q4);
                                     //q5
                                     double s5 = sin(q4)*Hz - cos(q4)*Hx;
                                     double c5 = Hy;
                                     q5 = angle_pi(atan2(s5, c5));
                                     joints_angles.push_back(q5);
                                     //std::cout << "initial solution for q5 is: " << q5 << std::endl;
                                     //q6
                                     double s6 = -cos(q4)*Fz - sin(q4)*Fx;
                                     double c6 = -cos(q4)*Gz - sin(q4)*Gx;
                                     q6 = angle_pi(atan2(s6, c6));
                                     joints_angles.push_back(q6);
                                     if(joints_angles.size() == 6){
                                         /*for (int k = 0; k < joints_angles.size(); ++k)
                                             std::cout << "commanded joint " << k << " angle is: " << joints_angles[k] << std::endl;
                                         std::cout << "**************************************" << std::endl;*/
                                         //check if all elements are within joints limits
                                         int count = 0;
                                         for(int t = 0; t < joints_angles.size(); ++t){
                                             if(joints_angles[t] > min_joint_limit(t) && joints_angles[t] < max_joint_limit(t)){
                                                 count += 1;
                                             }
                                             /*else
                                                 std::cout << "commanded joint " << t << " angle is: " << joints_angles[t]
                                                              << " and it is out of joints limits, minimus is: " << min_joint_limit(t)
                                                              << " and maximum is: " << max_joint_limit(t) << std::endl;*/
                                         }
                                         //std::cout << "**************************************" << std::endl;
                                         if(count == 6)
                                             final_joints_angles.push_back(joints_angles);
                                     }
                                     /*else
                                         std::cout << " no viable solution found the angles are: "
                                                   << q1 << ", " << q2 << ", " << q3 << ", "
                                                   << q4 << ", " << q5 << ", " << q6 << std::endl;*/
                                 }
                                 /*for (int j = 0; j < final_joints_angles.size(); ++j){
                                     for (int k = 0; k < final_joints_angles[j].size(); ++k)
                                         std::cout << "commanded joint " << k << " angle is: " << final_joints_angles[j][k] << std::endl;
                                 std::cout << "**************************************" << std::endl;
                                 }*/
                                 return final_joints_angles;
                                 }

//goto a cartesian position with different possible orientations and possibility to do push primitive after each trial using the option push
void Kinematics::goto_desired_position_with_all_orientations(std::vector<double> desired_position, bool push){
    std::vector<double> starting_joint_values = {-1.3, 0.3, -1.1, 0.0, -0.5, 0.0, 0.0, 0.0};
    goto_desired_joints_angles_position_mode(starting_joint_values);
    //find all solutions and all orientations
    std::vector<std::vector<double>> solutions = IG_complete(desired_position);
    //this is for case of one fixed orientation
    //std::vector<double> pose_fixed = {desired_position[0], desired_position[1], desired_position[2], 0, M_PI, 0};
    //std::vector<std::vector<double>> solutions = All_ig_solutions(pose_fixed);
    for(int i = 0; i < solutions.size(); ++i){
        /*std::cout << "trying to got to joints position: \n";
        for(int j = 0; j < solutions[i].size(); ++j)
            std::cout << solutions[i][j] << std::endl;*/
        bool main_motion = goto_desired_joints_angles_position_mode(solutions[i]);
        //std::cout << "****************************" << std::endl;
        //do the primitive push here
        usleep(1e6);
        if(main_motion && push){
            std::vector<double> pose = {desired_position[0], desired_position[1], desired_position[2], my_orientations[i][0], my_orientations[i][1], my_orientations[i][2]};
            //in case of one fixed orientation
            //std::vector<double> pose = {desired_position[0], desired_position[1], desired_position[2], 0, M_PI, 0};

            //execute primitive motion
            std::vector<double> push_solution = push_primitive(pose);
            if(!push_solution.empty()){
                goto_desired_joints_angles_position_mode(push_solution);
                usleep(1e6);
            }
        }

        //goto home position
        goto_desired_joints_angles_position_mode(starting_joint_values);
        usleep(1e6);
    }
    std::cout << "!!!!!!!!!!!!!!!! FINISHED !!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

//push primitive to be used with buttons modules
std::vector<double> Kinematics::push_primitive(std::vector<double> pose){
    double prim_distance = 0.07; //the small distance we want to end effector to push is 5 cm, we can change it till we are satisfied with the result
    Eigen::Vector4d target_position;
    target_position << 0, 0, prim_distance, 1;
    std::cout << "original angles: \n";
    std::cout << "Roll = " << pose[3] << std::endl
                 << "Pitch = " << pose[4] << std::endl
                    << "Yaw = " << pose[5] << std::endl;
    std::cout << "***************************************************" << std::endl;
    /*std::cout << "working with angles: \n";
    std::cout << "Roll = " << current_position[3] << std::endl
                 << "Pitch = " << current_position[4] << std::endl
                    << "Yaw = " << current_position[5] << std::endl;
    std::cout << "***************************************************" << std::endl;*/
    Eigen::Matrix3d rotation_matrix = Rot('z', pose[5]) * Rot('y', pose[4]) * Rot('x', pose[3]);
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix << rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2), pose[0],
                             rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2), pose[1],
                             rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2), pose[2],
                                                0,                    0,                    0,       1;

    target_position = transformation_matrix * target_position;
    std::vector<double> final_pose = {target_position[0], target_position[1], target_position[2], pose[3], pose[4], pose[5]};
    //std::cout << "the push primitive should give a position: \n" << target_position << std::endl;
    std::vector<std::vector<double>> solutions = All_ig_solutions(final_pose);
    if(!solutions.empty())
        return solutions[0];
    else{
        std::cout << "no primitive push solution !!!!!!!!!!!!!!!!!!!" << std::endl;
        std::vector<double> empty_vector;
        return empty_vector;
    }
}

void Kinematics::position_gripper(){
    std::vector<double> pos = {0.35f,-0.35f};
    unsigned char tmp_g[] = {8,9};
    std::vector<unsigned char> gripper_actuator_id (tmp_g, tmp_g + 2);
    Real robot;
    robot.getArm().set_joint_values(pos,gripper_actuator_id);
    robot.getArm().close_usb_controllers();
}

//Guides the real arm to desired joints angles by setting joints velocities to proper values each iteration till the duration time is reached
void Kinematics::goto_desired_joints_angles_velocity_mode(std::vector<double> desired_joints_angles)
{
    //this will be the velocity command send to the joints
    std::vector<double> joints_velocity(6), angles_distance(6);
    std::vector<double> joints_speeds(7);

    //instantiate a robot class to use for controlling the arm
    Real robot;

    /* get the distance to be covered by each joint by reading the current position and then subtract from it the desired joints angles
     * */
    /***************************check the distances may be they aren't what they supposed to be ********/
    std::vector<double> starting_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    for (int i = 0; i < 6; i++) {
        angles_distance[i] = desired_joints_angles[i] - M_PI * starting_angles[i];
    }

    //set the time, in seconds, to complete the trajectory in joints space
    double duration = 8;

    //set motors speed to zero so that they start from static situation
    for (int k = 0; k < reduced_actuator_id.size(); k++) {
        joints_speeds[k] = 0.0;
    }
    robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);

    //prepare motors to receive commands in velocity (change them to wheel mode)
    robot.getArm().mode_speed(reduced_actuator_id);

    //perform the trajectory in the desired time:

    //instantiate the interpolation parameter
    double rtdot;
    double time_passed = 0;
    // first initialize a timer:
    boost::timer time_here;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while (time_here.elapsed() < duration) {
        //at each iteration get the current value for the interpolation parameter
        rtdot = (30 * pow(time_here.elapsed(), 2)) / pow(duration, 3) -
                (60 * pow(time_here.elapsed(), 3)) / pow(duration, 4) +
                (30 * pow(time_here.elapsed(), 4)) / pow(duration, 5);

        //set joints velocities, by multiplying the rtdot interpolation parameter by the distance of joint (j)
        for (int j = 0; j < 6; j++) {
            joints_velocity[j] = rtdot * angles_distance[j];
        }
        //set joints velocities
        robot.getArm().set_joint_speeds(joints_velocity, reduced_actuator_id);
        //std::cout << "loop duration is: " << time_here.elapsed() - time_passed << std::endl;
        std::cout << "speed of important joint is: " << joints_velocity[2] * 60 << std::endl;
        time_passed = time_here.elapsed();
    }
    //std::cout << "final loop duration is: " << time_here.elapsed() << std::endl;
    starting_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    for (int i = 0; i < 6; i++) {
        std::cout << "finishing angle for joint: " << i << " is: " << M_PI * starting_angles[i] << std::endl;
    }
    //the joints should be now at their desired angles, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
}


std::vector<double> Kinematics::return_final_joints_values(std::vector<double> desired_position, std::vector<double>& joints_values){
    std::vector<double> joints_velocity,last_angles,my_angles;
    Real robot;

    //get the feedback about current joints angles, using the method defined in RobotArm.cpp, which actually returns the angle as a fraction of (pi), so we multiply it by (pi)
    //to get it in radian. After that we need to define it relevant to values given in initial_joint_values, as mentioned earlier.
    last_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < last_angles.size(); i++)
        my_angles.push_back(M_PI * last_angles[i]);
    for (int i = 0; i < last_angles.size(); i++) {
        last_angles[i] = M_PI * last_angles[i] - initial_joint_values[i];
    }


    //initialize the inverse kinematic to get the distance to be covered
    control_inverse_initialize(desired_position, last_angles);

    //first initialize a timer
    double my_time = 0.0, dt = 0.003;

    //Then for the specified duration calculate each iteration the proper joints velocities and set them
    while(my_time < duration)
    {

        /*solve for current desired joints speeds, and it appears that the return speeds are actually in RPS (Revolution Per Second) rather than radian/sec, more analysis is to
         * be carried out to see exactly why it gives this, but for the moment it simply means they don't any conversio except for converting them to RPM (Revolution Per Minute)
         * as this what the crustcrawler motors expect as speed command, this is done in the method (set_joint_speeds()) which belongs to robotArm class
        */
        joints_velocity = control_inverse(last_angles,duration,my_time);

        //at each iteration integrate the joint velocity to update joints angles for the inverse model so use the adapted angles
        for (int i = 0; i < last_angles.size(); i++)
            last_angles[i] = last_angles[i] + joints_velocity[i] * dt;
        //at each iteration integrate the joint velocity to update joints angles (the absolute one which the method shall return at the end
        for (int i = 0; i < my_angles.size(); i++){
            my_angles[i] = my_angles[i] + joints_velocity[i] * dt;
            joints_values.push_back(my_angles[i]);
        }
        my_time = my_time + dt;
    }
    //the end effector should be now at the desired position, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
    return my_angles;
}

//Guides the real arm to desired joints angles using position, the input is six desired angles
bool Kinematics::goto_desired_joints_angles_position_mode(std::vector<double> desired_joints_angles)
{
    //this will be the velocity command send to the joints
    std::vector<double> joints_speeds(7), starting_angles, q_des;

    //instantiate a robot class to use for controlling the arm
    Real robot;

    for (int i = 0; i < desired_joints_angles.size(); i++) {
        desired_joints_angles[i] = desired_joints_angles[i] / M_PI;
    }

    robot.getArm().mode_position(reduced_actuator_id);
    //set motors speeds to a desired speeds that will be used to reach the value of each joint
    for (int k = 0; k < reduced_actuator_id.size(); k++) {
        joints_speeds[k] = max_angular_speed;
    }
    robot.getArm().set_joint_speeds(joints_speeds, reduced_actuator_id);
    starting_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    q_des = starting_angles;
    joints_distance << desired_joints_angles[0] - starting_angles[0], desired_joints_angles[1] - starting_angles[1], desired_joints_angles[2] - starting_angles[2],
                       desired_joints_angles[3] - starting_angles[3], desired_joints_angles[4] - starting_angles[4], desired_joints_angles[5] - starting_angles[5];
    duration = 7.5*joints_distance.norm()/max_angular_speed;
    //boolean variable to know when the arm is overlaoded
    bool stressed = false;
    //avariable to read joints loads and show them
    std::vector <double> joints_loads;
    boost::timer my_timer;
    while(my_timer.elapsed()<duration){
        joints_loads = robot.getArm().get_joint_loads(reduced_actuator_id);
        for(int i = 0;i < joints_loads.size();i++){
            //std::cout << joints_loads[i] << std::endl;
            if (joints_loads[i] > max_load){
                stressed = true;
                break;
            }
        }
        if (stressed)
            break;
        rt = 10 * pow((my_timer.elapsed()/duration),3) - 15 * pow((my_timer.elapsed()/duration),4) + 6 * pow((my_timer.elapsed()/duration),5);
        for (int i = 0; i < q_des.size(); i++)
            q_des[i] = starting_angles[i] + rt * joints_distance[i];
        robot.getArm().set_joint_values(q_des, reduced_actuator_id);
    }
    //std::vector<double> finishing_angles = robot.getArm().get_joint_values(reduced_actuator_id);;
    //for (int i = 0; i < q_des.size(); i++) {
      //  std::cout << "finishing angle for joint: " << i << " is: " << M_PI * q_des[i] << std::endl;
    //}
    //if the arm collides with fixed obstacles stop the motion and releave the arm
    if (stressed){
        std::cout << "i am stressed ******************************" << std::endl;
        q_des[1] = 0; q_des[2] = -1.5/M_PI;
        robot.getArm().set_joint_values(q_des, reduced_actuator_id);
        stressed = false;
        return false;
    }
    //the joints should be now at their desired angles, so finish the program and close the crustcrawler communication bus
    robot.getArm().close_usb_controllers();
    return true;
}


//starting from the current position of the end effector this method will make the end effector goes for small distance in a forward sense
void Kinematics::primitive_motion(){
    std::vector<double> current_angles,current_position,target_position(3);
    double prim_distance = 0.05; //the small distance we want to end effector to push is 5 cm, we can change it till we are satisfied with the result

    Real robot;

    //get the current position, by getting current joint angles and then passing them to the forward model
    current_angles = robot.getArm().get_joint_values(reduced_actuator_id);
    for (int i = 0; i < current_angles.size(); i++) {
        current_angles[i] = M_PI * current_angles[i] - initial_joint_values[i];
    }
    current_position = forward_model(current_angles);

    //get the diagonal line from the origin to the end effector
    double angle = atan2(current_position[1], current_position[0]);

    //set the target position which will have the same z coordinate but x and y will change according to the direction defined above
    target_position[0] = current_position[0] + prim_distance * cos(angle);
    target_position[1] = current_position[1] + prim_distance * sin(angle);
    target_position[2] = current_position[2];

    std::vector<double> dummy;
    //perform the motion using the method goto_desired_position
    goto_desired_position_in_position_mode(target_position, dummy);
}
