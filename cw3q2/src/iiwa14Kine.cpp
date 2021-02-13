#include <cw3q2/iiwa14Kine.h>

void iiwa14_kinematic::init()
{
    X_alpha[0] = X_alpha[1] = X_alpha[2] = X_alpha[3] = X_alpha[4] = X_alpha[5] = M_PI_2; X_alpha[6] = 0.0;
    Y_alpha[0] = Y_alpha[1] = Y_alpha[3] = Y_alpha[5] = M_PI;
    Y_alpha[2] = Y_alpha[4] = Y_alpha[6] = 0.0;

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

    //The translation between each joint for manual forward kinematic (not using the DH convention).
    translation_vec.resize(7, 3);

    translation_vec << 0, 0, 0.2025,
            0, 0.2045, 0,
            0, 0, 0.2155,
            0, 0.1845, 0,
            0, 0, 0.2155,
            0, 0.081, 0,
            0, 0, 0.045;

    //The centre of mass of each link with respect to the preceding joint.
    link_cm.resize(7, 3);
    link_cm << 0, -0.03, 0.12,
            0.0003, 0.059, 0.042,
            0, 0.03, 0.13,
            0, 0.067, 0.034,
            0.0001, 0.021, 0.076,
            0, 0.0006, 0.0004,
            0, 0, 0.02;

    //The mass of each link.
    mass.resize(7);
    mass << 4, 4, 3, 2.7, 1.7, 1.8, 0.3;

    //Moment on inertia of each link, defined at the centre of mass.
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
  
    
    //joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);
}


void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    VectorXd J(7);

    for(int i = 0; i < 7; i++)
        J(i) = q->position.at(i);

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

//Transformation functions for forward kinematic.
Matrix4d T_translation(Vector3d t)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    for (int i = 0; i < 3; i++)
        T(i, 3) = t(i);
    return T;
}

//Transformation functions for forward kinematic.
Matrix4d T_rotationZ(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    return T;
}

//Transformation functions for forward kinematic.
Matrix4d T_rotationY(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 2) = sin(theta);
    T(2, 0) = -sin(theta);
    T(2, 2) = cos(theta);
    return T;
}


//Transformation functions for forward kinematic.
Matrix4d T_rotationX(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(1, 1) = cos(theta);
    T(1, 2) = -sin(theta);
    T(2, 1) = sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

Matrix4d iiwa14_kinematic::forward_kine(VectorXd joint_val, int frame)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    //Add offset from the iiwa platform.
    T(2, 3) = 0.1575;
    //Manual forward kine for dynamics purpose. This chain of transformation works exactly the same as forward kinematic.
    for (int i = 0; i < frame; i++)
        T = T * T_rotationZ(joint_val(i)) * T_translation(translation_vec.block<1, 3>(i, 0)) * T_rotationX(X_alpha[i]) * T_rotationY(Y_alpha[i]);

    return T;
}

MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete the question 1
    Matrix4d T = Matrix4d::Identity(4, 4);
    //Add offset from the iiwa platform.
    T(2, 3) = 0.1575;
    //Manual forward kine for dynamics purpose. This chain of transformation works exactly the same as forward kinematic.
    for (int i = 0; i < frame; i++)
        T = T * T_rotationZ(joint_val(i)) * T_translation(link_cm.block<1, 3>(i, 0)) * T_rotationX(X_alpha[i]) * T_rotationY(Y_alpha[i]);

    return T;
}

MatrixXd iiwa14_kinematic::get_jacobian(VectorXd joint_val)
{
    //TODO: Fill in this function to complete the question 1
    Matrix4d A;
    Vector3d Pe;
    Matrix4d T = Matrix4d::Identity(4, 4);
    MatrixXd J = MatrixXd::Constant(6,7,0.0);
    MatrixXd o = MatrixXd::Constant(3,8,0.0);
    MatrixXd z = MatrixXd::Constant(3,8,0.0);
    //first z is 001
    z(2,0) = 1;


    for(int i = 0;i < 7; i++)
    {
        A = forward_kine(joint_val, i+1);
        T = T * A;       
        //each loop extract value
        o(0,i+1)=T(0,3);o(1,i+1)=T(1,3);o(2,i+1)=T(2,3); 
        z(0,i+1)=T(0,2);z(1,i+1)=T(1,2);z(2,i+1)=T(2,2);
    }
      
    for(int i=0; i<7; i++) {
       //temperary read two vector
       Vector3d temo; 
       Vector3d temz;
       Vector3d Jp;
       Vector3d diff;

       Pe = o.block<3,1>(0,7);
       temo = o.block<3,1>(0,i);
       temz = z.block<3,1>(0,i);
       diff = Pe-temo;
       
       Jp=temz.cross(diff);
       J(0,i)=Jp(0); J(1,i)=Jp(1); J(2,i)=Jp(2);
       J(3,i)=temz(0); J(4,i)=temz(1); J(5,i)=temz(2);
    }
    
    return J;


}

MatrixXd iiwa14_kinematic::get_jacobian_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete the question 1
    
    Vector3d z_j = Vector3d::Zero(); 
    Vector3d p_j = Vector3d::Zero();
    Vector3d pli = Vector3d::Zero();
    Vector3d Jpj = Vector3d::Zero();

    MatrixXd Jp = MatrixXd::Constant(3,7,0.0);
    MatrixXd Jo = MatrixXd::Constant(3,7,0.0);
    MatrixXd J  = MatrixXd::Constant(6,7,0.0);

    Matrix4d T = Matrix4d::Constant(4,4,0.0);
    Matrix4d T_cm = Matrix4d::Constant(4,4,0.0);

    for(int i = 0;i < frame; i++)
    {
  
        if(i==0)//first 
          {
            z_j(2) = 1;
          }
        else
          {
            T = forward_kine(joint_val,i);  // find zj-1
            //each loop extract value
            z_j = T.block<3,1>(0, 2);
            p_j = T.block<3,1>(0, 3);
          }

        T_cm = forward_kine_cm(joint_val, i+1);
  
        pli = T_cm.block<3,1>(0, 3);
        
        Jpj = z_j.cross(pli-p_j);

        Jo(0,i) = z_j(0);  Jo(1,i) = z_j(1);  Jo(2,i) = z_j(2);

 
            J(0,i) = Jpj(0);  J(1,i) = Jpj(1);  J(2,i) = Jpj(2);
            J(3,i) = z_j(0);  J(4,i) = z_j(1);  J(5,i) = z_j(2);
       

    }

    return J;
}

VectorXd iiwa14_kinematic::inverse_kine_ite(Matrix4d pose, VectorXd joint_val)
{
    //TODO: Fill in this function to complete the question 1
    Matrix4d A;
    Matrix4d Tdesired;
    Matrix4d TNew;
    MatrixXd JNew;
    MatrixXd JPositionNew;
    MatrixXd JTranspose;
    Vector3d PDesired;
    Vector3d POld;
    VectorXd ThetaOld(7);
    VectorXd ThetaNew(7);
    VectorXd diff(7);
    VectorXd startangle(7);
    double x = pose(0, 3);
    double y = pose(1, 3);
    double z = pose(2, 3);
    //MatrixXd J = MatrixXd::Constant(3,5,0.0);  //pex,pey,pez as row, number of theta is col
    
    int k=1;
    int count=0;
    VectorXd Point_angle;
    double Threshold1=0.2;
    double Threshold2=0.2;
    

    //Tdesired = forward_kine(joint_val, 5);
    PDesired(0)=x;PDesired(1)=y;PDesired(2)=z;
    
 
    for(int i=0; i<7; i++) {
       startangle(i)=joint_val(i);
    }//save the desired sngle avoid overlapping later

    for(int i=0; i<7; i++) {
       joint_val(i)=startangle(i);
    } //initial first angle

    
    while(count<3 && k<10000){
          count=0;
          TNew = forward_kine(joint_val, 7); //find new T by updated angle
          JNew = get_jacobian(joint_val);           //call the function to find jacobian
          JPositionNew=JNew.block<3,7>(0,0);                 //only get the Jp
          JTranspose=JPositionNew.transpose(); //get the transpose of the jacobian
          POld(0)=TNew(0,3);POld(1)=TNew(1,3);POld(2)=TNew(2,3); //previous x,y,z
          
          for (int i = 0; i < 7; i++){  
              ThetaOld(i)=joint_val(i);
          }

          //update angle
          diff = JTranspose*(PDesired-POld);
          ThetaNew = ThetaOld-0.8*diff;  //0.8 is selected alpha(0~1)
          //update for new forward Kinetic

          for (int i = 0; i < 7; i++){  
              joint_val(i)=ThetaNew(i);
          }

          for (int i = 0; i < 3; i++){  
              if (abs(PDesired(i)-POld(i))<=Threshold1){  
                  count=count+1;
               }
          }

          k=k+1; //count
     }

    
    Point_angle=joint_val;

    return Point_angle;
}

MatrixXd iiwa14_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete the question 1. You may need to re-structure the input of this function.
    double coe = 0.3;
    double R = 0.05;
    double l = 0.145;
    double m1;
    double m2;
    double m3;
    double xe; double ye; double ze;
    double xw; double yw; double zw;
    Vector4d pwe;
    Vector3d pw;
    Vector3d pc;
    Vector3d pe;
    MatrixXd position;
    Matrix4d T4;
    Matrix4d T567;
    VectorXd joint_val;

    pwe(0) = link_cm(6,0)-link_cm(5,0);
    pwe(1) = link_cm(6,1)-link_cm(5,1);
    pwe(2) = link_cm(6,2)-link_cm(5,2);
    pwe(3) = 1.0;

    
    pw = (pose*pwe).block<3,1>(0,0);
    pe(0) = pc(0) + coe*R; pe(1) = pc(1) + coe*R; pe(2) = pc(2) + coe*R;

    xe = pe(0); ye=pe(1); ze=pe(2);
    xw = pw(0); yw=pw(1); zw=pw(2);
    
//find position 1-4
    position(0) = atan2(ye,xe);
    position(1) = atan2(sqrt(xe*xe+ye*ye),ze);
    m1 = xw*cos(position(1))*cos(position(0))+yw*sin(position(0))*cos(position(1))*zw*sin(position(1));
    m2 = xw*cos(position(0))*sin(position(1))*yw*sin(position(0))*sin(position(1))*zw*cos(position(1));
    m3 = xw*sin(position(0))+yw*cos(position(1));
    position(2) = atan2(-m3,m1);
    position(3) = atan2(sqrt(m1*m1+m3*m3),m2-l);
//find T4 and then find T567 T4*T567=POSE
    for (int i = 0; i < 4; i++){  
        joint_val(i) = position(i);
    }
    T4 = forward_kine_cm(joint_val,4);
    T567 = T4.inverse()*pose;
    position(4) = atan2(T567(1,2),T567(0,2));
    position(5) = atan2(sqrt(T567(1,2)*T567(1,2)+T567(0,2)*T567(0,2)),T567(2,2));
    position(6) = atan2(T567(2,0),-T567(2,1));
    
}


MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    //TODO: Fill in this function to complete the question 1
    double m;
    MatrixXd R = MatrixXd::Constant(3,3,0.0);
    MatrixXd I = MatrixXd::Identity(3,3);
    MatrixXd Jli = MatrixXd::Constant(3,3,0.0);
    MatrixXd Jp = MatrixXd::Constant(3,7,0.0);
    MatrixXd Jo = MatrixXd::Constant(3,7,0.0);
    MatrixXd B  = MatrixXd::Constant(7,7,0.0);
   


    for(int i = 0;i < 7; i++)
    { 
      Jp = get_jacobian_cm(joint_val,i+1).block<3,7>(0,0); //frame = i+1
      Jo = get_jacobian_cm(joint_val,i+1).block<3,7>(3,0);

      m = mass(i);

      R = forward_kine(joint_val,i+1).block<3,3>(0,0); 
      I(0,0) = 4*Ixyz(i,0); I(1,1) = 4*Ixyz(i,1); I(2,2) = 4*Ixyz(i,2);   
      Jli = R*I*R.transpose();

      B = B+(m*Jp.transpose()*Jp+ Jo.transpose()*Jli*Jo);
    }

     return B;
}

MatrixXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{
    //TODO: Fill in this function to complete the question 1
    int frame = 7;
    double bjk; double bij; double hijk; double cij = 0.0;
    MatrixXd B;
    MatrixXd C = MatrixXd::Constant(7,7,0.0);
    

    for(int i=0; i<frame; i++){
       for(int j=0; j<frame; j++){
           
           //f = x^2; 
           //bij = B(i,j);
           
           for(int k=0; k<frame; k++){
       // %%%%%%%%%%%%%%%%%%%%%%% diff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
               double  dx=0.01;    
               double  fx;
               double  fxe;
               VectorXd  x;
               VectorXd  xe;
               double  dd1;
               double  dd2;

               x = joint_val;
               x(k) = joint_limit_min[k];

               do{
                  B = getB(x);
                  fx = B(i,j);

                  xe = x;
                  xe(k) = x(k)+dx;
                  B = getB(xe);
                  fxe = B(i,j);

                  dd1=(fxe - fx)/dx;    
                  dx = 0.5 * dx;  
                  dd2=(fxe - fx)/dx;    

                  dx = 0.01;
                  x(k) = x(k)+dx;
                  }while (fabs(x(k)-joint_val(k)) >= 0.01); 
               bij = dd1; 
        

               x = joint_val;
               x(k) = joint_limit_min[k];


               do{
                  B = getB(x);
                  fx = B(j,k);

                  xe = x;
                  xe(k) = x(k)+dx;
                  B = getB(xe);
                  fxe = B(j,k);

                  dd1=(fxe - fx)/dx;    
                  dx = 0.5 * dx;  
                  dd2=(fxe - fx)/dx;    

                  dx = 0.01;
                  x(k) = x(k)+dx;
                  }while (fabs(x(k)-joint_val(k)) >= 0.01); 
       // %%%%%%%%%%%%%%%%%%%%%%% diff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
                bjk = dd1;
 

               hijk = bij-0.5*bjk;
               cij = cij + hijk*joint_vel(k);
              }


           cij = 0.0;
          }
    }

   return B;

}

VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    //TODO: Fill in this function to complete the question 1

    Vector3d g0; 
    double m;
    double P;
    Matrix4d T_cm = Matrix4d::Constant(4,4,0.0);
    Vector3d pli = Vector3d::Zero();  
    VectorXd G = VectorXd::Zero(7);

    P=0.0;
    g0(0) = 0; g0(1) = 0; g0(2) = g;
    
    
    for(int i = 0;i < 7; i++)
    {
  
       // Jp = get_jacobian_cm(joint_val,i+1).block<3,7>(0,0);
        
       // pli = Jp*joint_val;         //3by7 7by1 3by1

        m = mass(i);

       // P = P  - m*g0.transpose()*pli;  


        // %%%%%%%%%%%%%%%%%%%%%%% diff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               double  dx=0.01;    
               double  fx;
               double  fxe;
               VectorXd  x;
               VectorXd  xe;
               double  dd1;
               double  dd2;

               x = joint_val;
               x(i) = joint_limit_min[i];

               do{
                  T_cm = forward_kine_cm(x, i+1);
                  pli = T_cm.block<3,1>(0, 3);
                  fx = P  - m*g0.transpose()*pli; 

                  xe = x;
                  xe(i) = x(i)+dx;
                  T_cm = forward_kine_cm(xe, i+1);
                  pli = T_cm.block<3,1>(0, 3);
                  fxe = P  - m*g0.transpose()*pli; 

                  dd1=(fxe - fx)/dx;    
                  dx = 0.5 * dx;  
                  dd2=(fxe - fx)/dx;    

                  dx = 0.01;
                  x(i) = x(i)+dx;
                  }while (fabs(x(i)-joint_val(i)) >= 0.01); 
        // %%%%%%%%%%%%%%%%%%%%%%% diff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         G(i) = dd1;
    }


     return G;
}

  
