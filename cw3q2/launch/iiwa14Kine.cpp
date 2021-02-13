#include <cw3q2/iiwa14Kine.h>
#include <cmath>
#include <math.h>
#include "tf2_ros/transform_broadcaster.h"


void iiwa14_kinematic::init()
{
    //Delete this and fill DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro).
    // for (int i = 0; i < 7;i++){
    //     for (int j = 0; j < 4;j++){
    //         DH_params[i][j] = 0.0;
    //     }
    // }

    for (int j {0}; j < 7; j++){
        this->them_joints.push_back(0);
        this->them_joint_vels.push_back(0);
        this->them_joint_effs.push_back(0);
    }

    // second index is the a, alpha, d, theta
    // other idx is frames
    // A's
    DH_params[0][0] = 0;
    DH_params[1][0] = 0;
    DH_params[2][0] = 0;
    DH_params[3][0] = 0;
    DH_params[4][0] = 0;
    DH_params[5][0] = 0;
    DH_params[6][0] = 0;

    // alphas
    DH_params[0][1] = -M_PI / 2;
    DH_params[1][1] = M_PI / 2;
    DH_params[2][1] = M_PI / 2;
    DH_params[3][1] = -M_PI / 2;
    DH_params[4][1] = -M_PI / 2;
    DH_params[5][1] = M_PI / 2;
    DH_params[6][1] = 0;

    // dees
    DH_params[0][2] = 0.2025;
    DH_params[1][2] = 0;
    DH_params[2][2] = 0.42;
    DH_params[3][2] = 0;
    DH_params[4][2] = 0.4;
    DH_params[5][2] = 0;
    DH_params[6][2] = 0.126;

    // thetas
    DH_params[0][3] = 0;
    DH_params[1][3] = 0;
    DH_params[2][3] = 0;
    DH_params[3][3] = 0;
    DH_params[4][3] = 0;
    DH_params[5][3] = 0;
    DH_params[6][3] = 0;


    // translational parameters
    // THESE NEED TO CHANGE TO MATCH DH

    trans_params[0][0] = 0;
    trans_params[0][1] = -0.03;
    trans_params[0][2] = 0.12;

    trans_params[1][0] = -0.0003;
    trans_params[1][1] = -0.059;
    trans_params[1][2] = 0.042;

    trans_params[2][0] = 0;
    trans_params[2][1] = 0.03;
    trans_params[2][2] = 0.13 + 0.2045;

    trans_params[3][0] = 0;
    trans_params[3][1] = 0.067;
    trans_params[3][2] = 0.034;

    trans_params[4][0] = -0.0001;
    trans_params[4][1] = -0.021;
    trans_params[4][2] = 0.076 + 0.1845;

    trans_params[5][0] = 0;
    trans_params[5][1] = -0.0006;
    trans_params[5][2] = 0.0004;

    trans_params[6][0] = 0;
    trans_params[6][1] = 0;
    trans_params[6][2] = 0.02 + 0.081;



    joint_limit_min[0] = -170*M_PI/180;
    joint_limit_min[1] = -120*M_PI/180;
    joint_limit_min[2] = -170*M_PI/180;
    joint_limit_min[3] = -120*M_PI/180;
    joint_limit_min[4] = -170*M_PI/180;
    joint_limit_min[5] = -120*M_PI/180;
    joint_limit_min[6] = -175*M_PI/180;

    joint_limit_max[0] = 170*M_PI/180;
    joint_limit_max[1] = 120*M_PI/180;
    joint_limit_max[2] = 170*M_PI/180;
    joint_limit_max[3] = 120*M_PI/180;
    joint_limit_max[4] = 170*M_PI/180;
    joint_limit_max[5] = 120*M_PI/180;
    joint_limit_max[6] = 175*M_PI/180;

    //The mass of each link.
    // this is mli i guess
    mass.resize(7);
    mass << 4, 4, 3, 2.7, 1.7, 1.8, 0.3;

    //Moment on inertia of each link.
    //Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
    Ixyz.resize(7, 3);
    Ixyz << 0.1, 0.09, 0.02,
            0.05, 0.018, 0.044,
            0.08, 0.075, 0.01,
            0.03, 0.01, 0.029,
            0.02, 0.018, 0.005,
            0.005, 0.0036, 0.0047,
            0.001, 0.001, 0.001;

    //gravity
    g = 9.8;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/iiwa/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);
}


void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    VectorXd J(7);
    // std::cout << "Joints:\n";
    for(int i = 0; i < 7; i++){
        J(i) = q->position.at(i);
        this->them_joints.at(i) = q->position.at(i);
        this->them_joint_vels.at(i) = q->velocity.at(i);
        this->them_joint_effs.at(i) = q->effort.at(i);
    }
    // std::cout << std::endl << std::endl;

    current_pose = forward_kine(J, 7);
    broadcast_pose(current_pose);
}

Matrix4d iiwa14_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}

void iiwa14_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "iiwa_link_0";
    T.child_frame_id = "iiwa_ee";

    pose_br.sendTransform(T);
}

//Useful Transformation function.
Matrix4d T_rotationZ(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationY(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 2) = sin(theta);
    T(2, 0) = -sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationX(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(1, 1) = cos(theta);
    T(1, 2) = -sin(theta);
    T(2, 1) = sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

Matrix4d T_translation(Vector3d t)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    for (int i = 0; i < 3; i++)
        T(i, 3) = t(i);
    return T;
}

Vector3d iiwa14_kinematic::get_pose_pos(Matrix4d this_pose){
    Eigen::Vector3d zzeroPe;
    zzeroPe << (this_pose(0, 3)), this_pose(1, 3), this_pose(2,3);
    return zzeroPe;
}

// ==============================================================//
// Function for getting the orientation component of a
// homogeneous transformation matrix
// ==============================================================//
Vector3d iiwa14_kinematic::get_pose_ori(Matrix4d this_pose_ori){
    Eigen::Matrix3d identity;
    identity << 1,0,0, 0,1,0, 0,0,1;
    Eigen::Matrix3d the_rotation_m;
    double angle {acos((this_pose_ori(0,0) + this_pose_ori(1,1) + this_pose_ori(2,2) - 1) / 2)};
    double denom {sqrt(pow((this_pose_ori(2,1) - this_pose_ori(1,2)), 2) + pow((this_pose_ori(0,2) - this_pose_ori(2,0)), 2) + pow((this_pose_ori(1,0) - this_pose_ori(0,1)), 2))};
    Eigen::Vector3d to_be_returned;
    to_be_returned(0) = (this_pose_ori(2,1) - this_pose_ori(1,2)) / denom;
    to_be_returned(1) = (this_pose_ori(0,2) - this_pose_ori(2,0)) / denom;
    to_be_returned(2) = (this_pose_ori(1,0) - this_pose_ori(0,1)) / denom;
    return to_be_returned;
}

Matrix4d iiwa14_kinematic::forward_kine(VectorXd joint_val, int frames)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    //Add offset from the iiwa platform.
    T(2, 3) = 0.1575;
    //TODO: Fill in this function to complete Q2.
    for (int frame {0}; frame < frames; frame++){
        Matrix4d A;
        A = iiwa14_kinematic::dh_matrix_standard(this->DH_params[frame][0], this->DH_params[frame][1], this->DH_params[frame][2], this->them_joints.at(frame) + this->DH_params[frame][3]);
        T = T * A;
    }
    return T;
}

void iiwa14_kinematic::broadcast_DH_frames(){
    tf2_ros::TransformBroadcaster br;

    Eigen::Matrix4d T;
    T = Eigen::Matrix4d::Identity(4,4);
    Eigen::VectorXd jointz (7);
    jointz << this->them_joints.at(0), this->them_joints.at(1), this->them_joints.at(2), this->them_joints.at(3), this->them_joints.at(4), this->them_joints.at(5), this->them_joints.at(6);

    geometry_msgs::TransformStamped transform[8];

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id = transform[5].header.frame_id = transform[6].header.frame_id = transform[7].header.frame_id = "iiwa_link_0";

    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = transform[5].header.stamp = transform[6].header.stamp = transform[7].header.stamp = ros::Time::now();

    transform[0].child_frame_id = "iiwa_1";
    transform[1].child_frame_id = "iiwa_2";
    transform[2].child_frame_id = "iiwa_3";
    transform[3].child_frame_id = "iiwa_4";
    transform[4].child_frame_id = "iiwa_5";
    transform[5].child_frame_id = "iiwa_6";
    transform[6].child_frame_id = "iiwa_7";
    transform[7].child_frame_id = "iiwa_8";

    for (int trans {0}; trans < 7; trans++){
        Eigen::Matrix4d A;
        A = iiwa14_kinematic::forward_kine(jointz, trans);
        // T = T * A;

        Eigen::Affine3d T_affine;

        T_affine.matrix() = A;

        geometry_msgs::TransformStamped T_buffer = tf2::eigenToTransform(T_affine);
        transform[trans].transform = T_buffer.transform;
        br.sendTransform(transform[trans]);
    }
}
//
// MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
// {
//     //TODO: Fill in this function to complete Q2.
//     Matrix4d this_fr;
//     this_fr = iiwa14_kinematic::forward_kine(joint_val, frame);
//     Vector3d other_trans;
//     other_trans << this->trans_params[frame][0], this->trans_params[frame][1], this->trans_params[frame][2];
//     Matrix4d this_t;
//     Matrix4d rot;
//     rot = T_rotationZ(this->DH_params[frame][3]);
//     this_t = T_translation(other_trans);
//
//     return this_fr * rot * this_t;
// }

MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
    Matrix4d this_fr;
    this_fr = iiwa14_kinematic::forward_kine(joint_val, frame-1);
    Vector3d other_trans;
    other_trans << this->trans_params[frame-1][0], this->trans_params[frame-1][1], this->trans_params[frame-1][2];
    Matrix4d this_t;
    Matrix4d rot;
    rot = T_rotationZ(this->DH_params[frame-1][3] + this->them_joints.at(frame-1));
    this_t = T_translation(other_trans);

    return this_fr * rot * this_t;
}

void iiwa14_kinematic::broadcast_cm_frames(){
    tf2_ros::TransformBroadcaster br;

    Eigen::Matrix4d T;
    T = Eigen::Matrix4d::Identity(4,4);
    Eigen::VectorXd jointz (7);
    jointz << this->them_joints.at(0), this->them_joints.at(1), this->them_joints.at(2), this->them_joints.at(3), this->them_joints.at(4), this->them_joints.at(5), this->them_joints.at(6);

    geometry_msgs::TransformStamped transform[8];

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id = transform[5].header.frame_id = transform[6].header.frame_id = transform[7].header.frame_id = "iiwa_link_0";

    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = transform[5].header.stamp = transform[6].header.stamp = transform[7].header.stamp = ros::Time::now();

    transform[0].child_frame_id = "cm_iiwa_1";
    transform[1].child_frame_id = "cm_iiwa_2";
    transform[2].child_frame_id = "cm_iiwa_3";
    transform[3].child_frame_id = "cm_iiwa_4";
    transform[4].child_frame_id = "cm_iiwa_5";
    transform[5].child_frame_id = "cm_iiwa_6";
    transform[6].child_frame_id = "cm_iiwa_7";
    transform[7].child_frame_id = "cm_iiwa_8";

    for (int trans {1}; trans < 8; trans++){
        Eigen::Matrix4d A;
        A = iiwa14_kinematic::forward_kine_cm(jointz, trans);
        // T = T * A;

        Eigen::Affine3d T_affine;

        T_affine.matrix() = A;

        geometry_msgs::TransformStamped T_buffer = tf2::eigenToTransform(T_affine);
        transform[trans-1].transform = T_buffer.transform;
        br.sendTransform(transform[trans]);
    }
}

MatrixXd iiwa14_kinematic::get_jacobian(VectorXd joint_val)
{
    int num_joints {7};
    Eigen::MatrixXd zeroTe (4,4);
    Eigen::MatrixXd our_jacobian(6, num_joints);
    zeroTe = iiwa14_kinematic::forward_kine(joint_val, num_joints);
    Eigen::Vector3d zeroPe;
    zeroPe << (zeroTe(0, 3)), zeroTe(1, 3), zeroTe(2,3);
    Eigen::Vector3d oh;
    Eigen::Vector3d zedeye;
    for (int i{0}; i < num_joints; i++){
        Eigen::MatrixXd T (4,4);
        T = iiwa14_kinematic::forward_kine(joint_val, i);
        zedeye << T(0, 2), T(1,2), T(2,2);
        oh << T(0,3), T(1,3), T(2,3);
        Eigen::Vector3d brack;
        brack << (zeroPe(0) - oh(0)), (zeroPe(1) - oh(1)), (zeroPe(2) - oh(2));
        our_jacobian(3, i) = zedeye(0);
        our_jacobian(4, i) = zedeye(1);
        our_jacobian(5, i) = zedeye(2);
        Eigen::Vector3d top3rows;
        top3rows = zedeye.cross(brack);
        our_jacobian(0, i) = top3rows(0);
        our_jacobian(1, i) = top3rows(1);
        our_jacobian(2, i) = top3rows(2);
    }
    return our_jacobian;
}

MatrixXd iiwa14_kinematic::get_jacobian_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
    // P_li is the translational part of T * T_dyn
    // P_j-1 is the translational part of T
    // Z_j-1 is the z axis part of T

    MatrixXd jacob (6,7);

    for (int i{0}; i < 6; i++){
        for (int j {0}; j < 7; j++){
            jacob(i,j) = 0;
        }
    }

    Matrix4d T_cm;
    T_cm = iiwa14_kinematic::forward_kine_cm(joint_val, frame);

    Vector3d P_li;
    P_li << T_cm(0,3), T_cm(1,3), T_cm(2,3);

    for (int fr{0}; fr < frame; fr++){
        Matrix4d T_ncm;
        // if (fr == 0){
        //     T_ncm = Matrix4d::Identity(4,4);
        // } else {
        T_ncm = iiwa14_kinematic::forward_kine(joint_val, fr); // -1??
        // }

        Vector3d P_j_minus;
        P_j_minus << T_ncm(0, 3), T_ncm(1, 3), T_ncm(2, 3);
        Vector3d Z_j_minus;
        Z_j_minus << T_ncm(0,2), T_ncm(1,2), T_ncm(2,2);

        Vector3d diff;
        diff << (P_li(0) - P_j_minus(0)), (P_li(1) - P_j_minus(1)), (P_li(2) - P_j_minus(2));

        Vector3d crossed;
        crossed = Z_j_minus.cross(diff);
        // Jp
        jacob(0, fr) = crossed(0);
        jacob(1, fr) = crossed(1);
        jacob(2, fr) = crossed(2);

        // Jo
        jacob(3, fr) = Z_j_minus(0);
        jacob(4, fr) = Z_j_minus(1);
        jacob(5, fr) = Z_j_minus(2);
    }
    return jacob;
}


VectorXd iiwa14_kinematic::inverse_kine_ite(Matrix4d pose, VectorXd joint_vals)
{
    //TODO: Fill in this function to complete Q2.
    std::vector<double> eps;
    bool done {0};
    double alpha {0.3};
    Eigen::Vector3d desired_pose_p;
    desired_pose_p << pose(0, 3), pose(1,3), pose(2,3);
    Eigen::Vector3d desired_pose_r;
    desired_pose_r = iiwa14_kinematic::get_pose_ori(pose);
    float last_error {1000};
    std::vector<double> end_config;
    int count {0};
    std::vector<float> errors;
    int inc_alph {0};
    while (done == 0){
        // ==============================================================//
        // Initialise necessary variables
        // ==============================================================//
        Eigen::Matrix4d zeroTe;
        zeroTe = iiwa14_kinematic::forward_kine(joint_vals, 7);
        Eigen::Vector3d zeroPe;
        zeroPe = iiwa14_kinematic::get_pose_pos(zeroTe);
        Eigen::Vector3d zeroRe;
        zeroRe = iiwa14_kinematic::get_pose_ori(zeroTe);
        Eigen::MatrixXd full_diff_vec (6,1);

        // ==============================================================//
        // Compute error here
        // ==============================================================//
        double current_pos_err {sqrt(pow(desired_pose_p(0) - zeroPe(0),2) + pow(desired_pose_p(1) - zeroPe(1),2) + pow(desired_pose_p(2) - zeroPe(2),2))};
        double current_ori_err {sqrt(pow(desired_pose_r(0) - zeroRe(0),2) + pow(desired_pose_r(1) - zeroRe(1),2) + pow(desired_pose_r(2) - zeroRe(2),2))};
        bool conv;
        conv = (last_error == (sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2))));

        // ==============================================================//
        // Loop condition below!
        // ==============================================================//
        if (((sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2))) < 0.0001) ){
            done = 1;
            std::cout << "Converged in " << count << " iterations!\n";
            for (int i {0}; i < 7; i++){
                end_config.push_back(joint_vals(i));
            }
            std::cout << "Current error: \t" << (sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2))) << std::endl;
        } else {
            if ((count % 1000) == 0){
                std::cout << "Current error: \t" << (sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2))) << std::endl;// << "\nLast error: \t" << last_error << std::endl;
                std::cout << "Last error: \t" << last_error << std::endl;
                std::cout << (static_cast<float>(sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2))) == last_error) << std::endl;
                last_error = (sqrt(pow(current_pos_err, 2) + pow(current_ori_err, 2)));
                errors.push_back(last_error);
                if (errors.size() > 5){
                    float sum {0};
                    for (int back {2}; back < 7; back++){
                        sum = sum + errors.at(errors.size()-back);
                    }
                    sum = sum / 5;
                    // ==============================================================//
                    // Increase the learning rate if the error does not change
                    // ==============================================================//
                    if ((last_error < (sum + 0.01)) && (last_error > (sum - 0.01))){
                        inc_alph++;
                    } else {
                        inc_alph = 0;
                    }
                    // ==============================================================//
                    // Another loop condition as aforementioned
                    // ==============================================================//
                    if (inc_alph >= 5){
                        done = 1;
                        std::cout << "Converged in " << count << " iterations!\n";
                        for (int i {0}; i < 7; i++){
                            end_config.push_back(joint_vals(i));
                        }
                    }
                }
                std::cout << "Increment by: " << inc_alph << std::endl;
            }
            // ==============================================================//
            // Vectors for positional and rotational difference
            // ==============================================================//
            Eigen::Vector3d p_diff;
            p_diff =  desired_pose_p - zeroPe;
            Eigen::Vector3d r_diff;
            r_diff = desired_pose_r - zeroRe;

            full_diff_vec << p_diff(0), p_diff(1), p_diff(2), r_diff(0), r_diff(1), r_diff(2);

            // ==============================================================//
            // Compute Jacobian
            // ==============================================================//
            Eigen::MatrixXd this_jacob (6, 7);
            this_jacob = iiwa14_kinematic::get_jacobian(joint_vals);

            // ==============================================================//
            // Adjust joint values as described in report
            // ==============================================================//
            Eigen::MatrixXd theta_dot = (alpha + (0.1*inc_alph)) * (this_jacob.transpose() * full_diff_vec);
            for (int i {0}; i < 7; i++){
                joint_vals(i) = (joint_vals(i) + theta_dot(i));
            }
        }
        count++;
    }

    // ==============================================================//
    // Apply the necessary modulus to the final configuration
    // ==============================================================//
    for (int i {0}; i < 7; i++){
        end_config.at(i) = fmod(end_config.at(i), M_PI*2);
    }

    VectorXd comp(7);

    for (int j{0}; j < 7; j++){
        comp(j) = end_config.at(j);
    }

    return comp;
}

MatrixXd iiwa14_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    MatrixXd bee(7,7);
    for(int x{0}; x < 7; x++){
        for (int y{0}; y < 7; y++){
            bee(x,y) = 0;
        }
    }
    for(int k {1}; k < 8; k++){

        //=========================================================================================//
        //=========================================================================================//``
        // Get jacobian and Jp Jo here
        MatrixXd this_jack(6,7);
        this_jack = iiwa14_kinematic::get_jacobian_cm(joint_val, k);

        MatrixXd jaypee (3,7);
        MatrixXd jayoh (3,7);

        jaypee << this_jack(0,0), this_jack(0,1), this_jack(0,2), this_jack(0,3), this_jack(0,4), this_jack(0,5), this_jack(0,6),       this_jack(1,0), this_jack(1,1), this_jack(1,2), this_jack(1, 3), this_jack(1,4), this_jack(1,5), this_jack(1,6),       this_jack(2, 0), this_jack(2,1), this_jack(2,2), this_jack(2,3), this_jack(2,4), this_jack(2,5), this_jack(2,6);
        jayoh << this_jack(3,0), this_jack(3,1), this_jack(3,2), this_jack(3,3), this_jack(3,4), this_jack(3,5), this_jack(3,6),       this_jack(4,0), this_jack(4,1), this_jack(4,2), this_jack(4, 3), this_jack(4,4), this_jack(4,5), this_jack(4,6),       this_jack(5, 0), this_jack(5,1), this_jack(5,2), this_jack(5,3), this_jack(5,4), this_jack(5,5), this_jack(5,6);

        //=========================================================================================//
        //=========================================================================================//
        // Getting rotational part and squiggly I here (need to work out how to do the Io thing)
        Matrix3d rot_part;
        Matrix4d T;
        T = iiwa14_kinematic::forward_kine_cm(joint_val, k);

        rot_part << T(0,0), T(0,1), T(0,2), T(1,0), T(1,1), T(1,2), T(2,0), T(2,1), T(2,2);
        Matrix3d squiggly_eye = Matrix3d::Identity(3,3);
        squiggly_eye(0,0) = this->Ixyz(k-1, 0);
        squiggly_eye(1,1) = this->Ixyz(k-1, 1);
        squiggly_eye(2,2) = this->Ixyz(k-1, 2);
        Matrix3d squigg;
        squigg = rot_part * squiggly_eye * rot_part.transpose();

        //=========================================================================================//
        //=========================================================================================//

        double this_mass;
        this_mass = this->mass(k-1);
        MatrixXd to_add (7,7);
        to_add = (this->mass(k-1) * (jaypee.transpose() * jaypee)) + (jayoh.transpose() * squigg * jayoh);

        bee = bee + to_add;
    }
    for(int x{0}; x < 7; x++){
        for (int y{0}; y < 7; y++){
            if((bee(x,y)*bee(x,y)) < 0.000001){
                bee(x,y) = 0;
            }
        }
    }
    return bee;
}

MatrixXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{
    //TODO: Fill in this function to complete Q2.
    double epsilon {0.000001};

    MatrixXd cee(7,7);

    for (int row {0}; row < 7; row++){
        for (int col {0}; col < 7; col++){
            double add_me {0};

            for (int joint {0}; joint < 7; joint++){
                MatrixXd actual_bee (7,7);
                actual_bee = iiwa14_kinematic::getB(joint_val);

                MatrixXd beep1 (7,7);
                VectorXd adj_j_1 (7);
                adj_j_1 = joint_val;
                adj_j_1(joint) = adj_j_1(joint) + epsilon;
                beep1 = iiwa14_kinematic::getB(adj_j_1);


                double first_term;
                first_term = (beep1(row, col) / epsilon) - (actual_bee(row, col) / epsilon);


                MatrixXd beep2 (7,7);
                VectorXd adj_j_2;
                adj_j_2 = joint_val;
                adj_j_2(row) = adj_j_2(row) + epsilon;
                beep2 = iiwa14_kinematic::getB(adj_j_2);

                double second_term;
                second_term = 0.5 * ((beep2(col, joint) / epsilon) - (actual_bee(col, joint) / epsilon));

                double hijk {first_term - second_term};

                add_me = add_me + (hijk * joint_vel(joint));
            }
            cee(row, col) = add_me;
        }
    }
    return cee;
}

VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.

    // what is that bold G?
    // bold g is vector [0, 0, 9.8]

    double epsilon {0.00001};

    Vector3d lil_g;
    lil_g << 0, 0, -9.81;

    VectorXd gee(7);
    gee << 0, 0, 0, 0, 0, 0, 0;

    for (int fr{0}; fr < 7; fr++){
        MatrixXd lil_jac (6,7);
        lil_jac = iiwa14_kinematic::get_jacobian_cm(joint_val, fr+1);

        MatrixXd jaypee (3,7);

        for (int i{0}; i < 3; i++){
            for (int j {0}; j < 7; j++){
                jaypee(i,j) = lil_jac(i,j);
            }
        }

        VectorXd hhh(7);
        hhh = -this->mass(fr) * (lil_g.transpose() * jaypee);
        gee = gee + hhh;

    }

    return gee;
}
