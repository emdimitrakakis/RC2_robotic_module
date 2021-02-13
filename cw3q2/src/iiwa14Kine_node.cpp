#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include "cw3q2/iiwa14Kine.h"
#include "random"
#include "chrono"

VectorXd generate_position()
{
    VectorXd random_joint;

    random_joint.resize(7);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> dist(0.0, 5*M_PI/9);

    for (int i = 0; i < 7; i++)
        random_joint(i) = dist(generator);

    return random_joint;
}

VectorXd generate_velocity()
{
    VectorXd random_joint;

    random_joint.resize(7);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> dist(0.0, M_PI/15);

    for (int i = 0; i < 7; i++)
        random_joint(i) = dist(generator);

    return random_joint;
}


void iiwa14_kinematics() {

    Eigen::Matrix4d A;
    //double curr_joint[5], joint1_values[5], joint2_values[5], joint3_values[5], joint4_values[5], joint5_values[5];

    Matrix4d T;
    Eigen::MatrixXd T_cm, T_ite, jacobian, jacobian_cm, IK_closed, B, C, G;
    Eigen::VectorXd IK_ite, p0(7), p1(7), v1(7), p2(7), v2(7), p3(7), v3(7);
	
	iiwa14_kinematic object;
	object.init();
	
	p0 << 0, 0, 0, 0, 0, 0, 0;
	//first test point
	p1 << 0.768506, 0.121759, -0.939627, 1.99111, -0.200346, -1.35671, -0.790215;
	v1 << -0.033846, 0.0564997, 0.295151, 0.271778, 0.259246, 0.0124612, -0.0292725;
	//second test point
	p2 << -1.58425, -0.272908, 0.796272, 2.37887, 0.0842769, -0.0436095, 1.05386;
	v2 << -0.327998, -0.166912, 0.147049, -0.339939, -0.0167218, -0.272989, 0.0179721;
	//third test point
	p3 << -2.13286, 0.266082, 0.942719, 2.37668, -2.04084, -0.598984, -0.832711;
	v3 << 0.203643, 0.130777, 0.0819627, 0.181411, 0.033077, -0.00593176, 0.0985884;


	T_cm = object.forward_kine_cm(p1,7);
	jacobian = object.get_jacobian(p1);
	jacobian_cm = object.get_jacobian_cm(p1,7);
	B = object.getB(p1);
	C = object.getC(p1, v1);
	G = object.getG(p1);

	std::cout << "///// JOINT 1 /////" << std::endl;
	std::cout << "Position: \n" << "[" << p1[0] << ", " << p1[1] << ", " << p1[2] << ", " << p1[3] << ", " << p1[4] << ", " << p1[5] << ", " << p1[6] << "]" << "\n" << std::endl;
	std::cout << "Velocity: \n" << "[" << v1[0] << ", " << v1[1] << ", " << v1[2] << ", " << v1[3] << ", " << v1[4] << ", " << v1[5] << ", " << v1[6] << "]" << "\n" << std::endl;
	std::cout << "T_cm is: \n"<< T_cm << "\n" << std::endl;
	std::cout << "The Jacobian is: \n"<< jacobian << "\n" << std::endl;
	std::cout << "The Jacobian at the cm is: \n"<< jacobian_cm << "\n" << std::endl;
	std::cout << "The iterative form IK is: \n"<< IK_ite << std::endl;
	std::cout << "B: \n"<< B << "\n" << std::endl;
	std::cout << "C: \n"<< C << "\n" << std::endl;
	std::cout << "G: \n"<< G << "\n" << std::endl;
	std::cout << "\n" << std::endl;

}

int main(int argc, char **argv)
{	
    ros::init(argc, argv, "cw3q2_node");
    //ros::NodeHandle nh;

    iiwa14_kinematics();
    ros::spin();
}
