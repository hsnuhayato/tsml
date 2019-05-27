#include <stdio.h>
#include <fstream>
#include <math.h>
#include "commonFunc.h"
// static std::ofstream ofs("/home/player/tsml/log/tar.log");
// static std::ofstream ofs2("/home/player/tsml/log/tarq.log");

double vel(double* Array,int i)
{
  double vel=0;
  vel=(Array[i+1]-Array[i-1])/0.01;
  return vel;
}

double acc(double* Array,int i)
{
  double acc=0;
  acc=(Array[i-1]-2*Array[i]+Array[i+1])/(0.005*0.005);
  return acc;
}

double sum(double* Array1st,int n)
{
  double total=0;
  for(int i=0;i<n;i++){
    //cout<<"AR "<<Array1st[i]<<endl;
    total+=Array1st[i];}
  return total;
}
////for robot..///
void getModelAngles(const BodyPtr body, TimedDoubleSeq &m_refq)
{
  int dof=body->numJoints();
  for(unsigned int i=0;i<dof;i++)
    m_refq.data[i]=body->joint(i)->q();
}

//new
void update_model(const BodyPtr body, const TimedDoubleSeq &m_q, const FootType FT, const string *end_link)
{
  int dof = body -> numJoints();
  for(unsigned int i=0;i<dof;i++)
    body->joint(i)->q()=m_q.data[i];

  JointPathPtr Lf2Rf= getCustomJointPath(body, body->link(end_link[LLEG]),body->link(end_link[RLEG]));
  JointPathPtr Rf2Lf= getCustomJointPath(body, body->link(end_link[RLEG]),body->link(end_link[LLEG]));
  
  if((FT==FSRFsw)||FT==RFsw){
    //body->link("LLEG_JOINT5")->p()=p_Init[LLEG];
    //body->link("LLEG_JOINT5")->R()=R_Init[LLEG];//tvmet::identity<matrix33>();
    Lf2Rf->calcForwardKinematics();
  }
  else if((FT==FSLFsw)||FT==LFsw){
    //body->link("RLEG_JOINT5")->p()=p_Init[RLEG];
    //body->link("RLEG_JOINT5")->R()=R_Init[RLEG];//tvmet::identity<matrix33>();
    Rf2Lf->calcForwardKinematics();
  }
  body->calcForwardKinematics();
}


Matrix3 rotationZ(const double theta){
  return Matrix3(AngleAxisd(theta, Vector3d::UnitZ()));
}

Matrix3 rotationY(const double theta){
  return Matrix3(AngleAxisd(theta, Vector3d::UnitY()));
}


void get_end_link_pose(Position* pose, const BodyPtr body, const string *end_link)
{
  //RLEG, LLEG, RARM, LARM, WAIST,
  for(int i=0; i<LINKNUM; i++) {
    pose[i].linear() = body->link(end_link[i])->R();
    pose[i].translation() = body->link(end_link[i])->p();
  }
}

/*
Vector3 calcZMP(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor)
{
  matrix33 w_R_rf,w_R_lf;
  Vector3 w_P_rf,w_P_lf;
  Vector3 wZMP_R,wZMP_L, wZMP;
  Vector3 rZMP_R,lZMP_L;
  double d = 0.01;//足底からセンサ中心の距離（適当に0.01[m]とした）
  rZMP_R << (-m_rfsensor.data[4] - m_rfsensor.data[0]*d)/m_rfsensor.data[2],
           ( m_rfsensor.data[3] - m_rfsensor.data[1]*d)/m_rfsensor.data[2],
            -0.135;
  lZMP_L << (-m_lfsensor.data[4] - m_lfsensor.data[0]*d)/m_lfsensor.data[2],
           ( m_lfsensor.data[3] - m_lfsensor.data[1]*d)/m_lfsensor.data[2],
            -0.135;
  
    w_P_rf=body->link("RLEG_JOINT5")->p();
    w_P_lf=body->link("LLEG_JOINT5")->p();
    // w_P_lf(i)=body->link("LLEG_JOINT5")->p()[i];
    w_R_rf=body->link("RLEG_JOINT5")->R;
    w_R_lf=body->link("LLEG_JOINT5")->R;
    
    wZMP_R = w_P_rf + w_R_rf*rZMP_R;
    wZMP_L = w_P_lf + w_R_lf*lZMP_L;
    
    wZMP[0] = (wZMP_R(0)*m_rfsensor.data[2] + wZMP_L(0)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP[1] = (wZMP_R(1)*m_rfsensor.data[2] + wZMP_L(1)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP[2] = wZMP_R(2);
  
  return wZMP;
}
*/

 /*
TimedDoubleSeq calcZMPTDS(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor)
{
  matrix33 w_R_rf,w_R_lf;
  Vector3 w_P_rf,w_P_lf;
  Vector3 wZMP_R,wZMP_L;
  Vector3 rZMP_R,lZMP_L;
  TimedDoubleSeq wZMP;
  double d = 0.01;//足底からセンサ中心の距離（適当に0.01[m]とした）

  wZMP.data.length(3); 
  rZMP_R << (-m_rfsensor.data[4] - m_rfsensor.data[0]*d)/m_rfsensor.data[2],
           ( m_rfsensor.data[3] - m_rfsensor.data[1]*d)/m_rfsensor.data[2],
            -0.135;
  lZMP_L << (-m_lfsensor.data[4] - m_lfsensor.data[0]*d)/m_lfsensor.data[2],
           ( m_lfsensor.data[3] - m_lfsensor.data[1]*d)/m_lfsensor.data[2],
            -0.135;
  
    w_P_rf=body->link("RLEG_JOINT5")->p();
    w_P_lf=body->link("LLEG_JOINT5")->p();
    // w_P_lf(i)=body->link("LLEG_JOINT5")->p()[i];
    w_R_rf=body->link("RLEG_JOINT5")->R;
    w_R_lf=body->link("LLEG_JOINT5")->R;
    
    wZMP_R = w_P_rf + w_R_rf*rZMP_R;
    wZMP_L = w_P_lf + w_R_lf*lZMP_L;
    
    wZMP.data[0] = (wZMP_R(0)*m_rfsensor.data[2] + wZMP_L(0)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP.data[1] = (wZMP_R(1)*m_rfsensor.data[2] + wZMP_L(1)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP.data[2] = wZMP_R(2);
  
  return wZMP;

}
 */
/*
void calcJs(BodyPtr body, MatrixXd &Js)
{
  int dof=body->numJoints();
  Js=Eigen::MatrixXd::Identity(dof,dof);
  int id = body->link("CHEST_JOINT0")->jointId;
  Js(id,id)=0;
  id = body->link("CHEST_JOINT1")->jointId;
  Js(id,id)=0;
  id = body->link("HEAD_JOINT0")->jointId;
  Js(id,id)=0;
  id = body->link("HEAD_JOINT1")->jointId;
  Js(id,id)=0;
  //ShowMatrix(Js, dof, dof);
}
*/


int swLeg(FootType FT)
{
  //swingLeg
    int swingLeg;
    if((FT==FSRFsw)||(FT==RFsw)){
      return RLEG;
    }
    else if((FT==FSLFsw)||(FT==LFsw)){
      return LLEG;
    }

}

bool loadtxt(std::string path,  std::deque<double> &data)
{
  ifstream file(path.c_str());
  if( !file )
    return false;

  string st;
  while(getline(file, st)) {
    if( st.size() > 1 ) {
      stringstream ss;
      ss.str(st);
      double tmp;
      ss >> tmp;
      data.push_back(tmp);
      //ofs<<tmp<<endl;
    }
  }
  file.close();

  return true;
}

Vector3 velicity(std::deque<Vector3> &deq, int i)
{
  Vector3 vel=Vector3d::Zero(3);
  vel=(deq.at(i+1)-deq.at(i-1))/0.01;
  return vel;
}

Vector3 acclar(std::deque<Vector3> &deq, int i)
{
  Vector3 acc=Vector3d::Zero(3);
  acc=(deq.at(i-1)-2*deq.at(i)+deq.at(i+1))/(0.005*0.005);
  return acc;
}


FootType FTselect(BodyPtr body,StepDir dir)
{
  FootType FT;
  //rf is front //initial pos rleg is behind lleg
  if( body->link("RLEG_JOINT5")->p()(0) >= body->link("LLEG_JOINT5")->p()(0) ){
    if(dir==front)
      FT=FSLFsw;
    else
      FT=FSRFsw;
  }
  else{//lf is front
     if(dir==front)
      FT=FSRFsw;
    else
      FT=FSLFsw;
  }

  return FT;

}

//----------------------------------------------------------------------------------
//  math function
//----------------------------------------------------------------------------------
/*
hrp::dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2)
{
  int size1 = in1.rows() + in2.rows();
  int size2 = in1.cols() + in2.cols();

  hrp::dmatrix out(Eigen::MatrixXd::Zero(size1,size2)); 
  
  for(int i = 0; i < in1.rows(); i++)
    for(int j = 0; j < in1.cols(); j++)
      out(i,j) = in1(i,j);


  int shift1 = in1.rows();
  int shift2 = in1.cols();
  for(int i = 0; i < in2.rows(); i++)
    for(int j = 0; j < in2.cols(); j++)
      out(i+shift1,j+shift2) = in2(i,j);

  return out;
}

hrp::dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2, const hrp::dmatrix &in3, const hrp::dmatrix &in4)
{
  int size1 = in1.rows() + in2.rows()+in3.rows() + in4.rows();
  int size2 = in1.cols() + in2.cols() + in3.cols() + in4.cols();

  hrp::dmatrix out(Eigen::MatrixXd::Zero(size1,size2)); 
   
  for(int i = 0; i < in1.rows(); i++)
    for(int j = 0; j < in1.cols(); j++)
      out(i,j) = in1(i,j);


  int shift1 = in1.rows();
  int shift2 = in1.cols();
  for(int i = 0; i < in2.rows(); i++)
    for(int j = 0; j < in2.cols(); j++)
      out(i+shift1,j+shift2) = in2(i,j);

  shift1 += in2.rows();
  shift2 += in2.cols();
  for(int i = 0; i < in3.rows(); i++)
    for(int j = 0; j < in3.cols(); j++)
      out(i+shift1,j+shift2) = in3(i,j);

  shift1 += in3.rows();
  shift2 += in3.cols();
  for(int i = 0; i < in4.rows(); i++)
    for(int j = 0; j < in4.cols(); j++)
      out(i+shift1,j+shift2) = in4(i,j);


  return out;
}
*/

Vector3 rot2rpy(Matrix3 R)
{
  Vector3 euler =R.eulerAngles(2,1,0);
  Vector3 out(euler(2), euler(1), euler(0));

  return out;
}

// todo vecotr cross
matrix22 RfromMatrix3(Matrix3 Rin)//bimyou
{
  //Vector3 rpy=rpyFromRot(Rin);
  //Matrix3 R=rotationZ(rpy(2));
  Matrix3 R=extractYow(Rin);
  matrix22 R_xy;
  for (int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      R_xy(i,j)=R(i,j);
    }
  }

  //R_xy = R.block<2,2>(0,0);
  return R_xy;
}

Matrix3 extractYow(Matrix3 Rin)
{
  Vector3 rpy = rpyFromRot(Rin);
  Matrix3 R = rotationZ(rpy(2));
  // Matrix3 R=rotationZ(euler(0));

  return R;
}

//please use calcWaistR in zmp planner
void clacWaistR(BodyPtr body, FootType FT,Matrix3&  R_ref_WAIST, double swLegYow)
{
  Vector3 sup_leg_rpy;
  Matrix3 sup_leg_R; 
  if((FT==FSRFsw)||(FT==RFsw)){
    sup_leg_rpy=rpyFromRot(body->link("LLEG_JOINT5")->R());
    sup_leg_R=body->link("LLEG_JOINT5")->R();
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    sup_leg_rpy=rpyFromRot(body->link("RLEG_JOINT5")->R());
    sup_leg_R=body->link("RLEG_JOINT5")->R();
  } 

  double YowAvg= (swLegYow +  sup_leg_rpy(2))/2;
  R_ref_WAIST=rotationZ(YowAvg);
 
}

void copy_poses(Position* pose_copy, const Position* const pose)
{
  for(int i=0; i<LINKNUM; i++) {
    pose_copy[i] = pose[i];
  }
}

void adjust_M_PI(double &v)
{
  while(true) {
    if( v > M_PI )
      v -= ( 2 * M_PI );
    else if( v < -M_PI )
      v += ( 2 * M_PI );
    else
      break;
  }
  
}

void atan2adjust(Vector3 &pre, Vector3 &cur)
{
  for(int i=0;i<3;i++){
    if(fabs(pre(i)- cur(i))>M_PI){
      if(pre(i) > cur(i) ){
        while(fabs(pre(i) - cur(i))>M_PI)
          cur(i)+=2*M_PI;
      }
      else if(pre(i) < cur(i) ){
        while(fabs(pre(i) - cur(i))>M_PI)
          cur(i)-=2*M_PI;
      }   
    }// if(fabs(pre - cur)>M_PI)
  }//for

  pre=cur;
}


bool CalcIVK_biped(const BodyPtr body, const Vector3& CM_p, const Position* pose_ref, const FootType& FT, const string *end_link)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg,SupLeg2W;
  Vector3 cm=body->calcCenterOfMass();
  Vector3  SwLeg_p; 
  Matrix3 W_R,SwLeg_R; 
 
  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  //careful
  MatrixXd Jacobian=MatrixXd::Zero(12,12);//leg only
  int swingLegId;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
    swingLegId=0;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
    swingLegId=1;
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  //new
  Vector3 SupLeg_p_Init;
  Matrix3 SupLeg_R_Init; 
  SupLeg_p_Init=SupLeg->p();
  SupLeg_R_Init=SupLeg->R();
  
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = 12;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < 12; ++i){
      qorg[i] = body->joint(i)->q();
    }
    
    //careful
    VectorXd dq(n);
    VectorXd v(12);
    double maxIKErrorSqr=1.0e-16;
    
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      W_R=body->link(end_link[WAIST])->R();
      SwLeg_p = SwLeg->p();
      SwLeg_R = SwLeg->R();
      //tag
      CalJo_biped(body, FT, Jacobian, end_link);        
      ///

      Vector3 CM_dp=CM_p- body->calcCenterOfMass();// + body->link("LLEG_JOINT5")->p);
      Vector3 W_omega=W_R* omegaFromRot(W_R.transpose() * pose_ref[WAIST].linear());
      Vector3 SW_dp =pose_ref[swingLegId].translation() - SwLeg_p;
      Vector3 SW_omega = SwLeg_R* omegaFromRot(SwLeg_R.transpose() * pose_ref[swingLegId].linear());
     
      //v<< CM_dp, W_omega, SW_dp, SW_omega;
      v.head<3>()=CM_dp;
      v.segment<3>(3)=W_omega;
      v.segment<3>(6)=SW_dp;
      v.segment<3>(9)=SW_omega;

      //double errsqr =  CM_dp.dot(CM_dp) + W_omega.dot(W_omega) + SW_dp.dot(SW_dp) + SW_omega.dot( SW_omega);
      double errsqr=v.squaredNorm();

      if(errsqr < maxIKErrorSqr){
        converged = true;
        break;
      }
      
      MatrixXd JJ;
      double dampingConstantSqr=1e-12;
      JJ = Jacobian *  Jacobian.transpose() + dampingConstantSqr * MatrixXd::Identity(Jacobian.rows(), Jacobian.rows());
      dq = Jacobian.transpose() * QR.compute(JJ).solve(v);


      for(int j=0; j < n; ++j){
        body->joint(j)->q() += LAMBDA * dq(j);
      }

      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
      
    }//for
    
    if(!converged){
      
      for(int j=0; j < n; ++j){
        body->joint(j)->q() = qorg[j];
      }

      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
    }
    
    return converged;          
}


//////ivk_jacobian/////////
void CalJo_biped(const BodyPtr body, const FootType& FT, Eigen::MatrixXd& out_J, const string *end_link)
{ 
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  MatrixXd JSupLeg2SwLeg ,JSupLeg2W,Jcom;

  
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  body->calcCenterOfMass();
  calcCMJacobian(body, SupLeg, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
  
  
  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      out_J(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId();
    for(int j=3;j<6;j++){
      out_J(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId();
    for(int j=6;j<12;j++){
      out_J(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
  
}


Matrix3 rodoriges(Vector3 omega, double dt)
{
  //Vector3 omega_norm 
  double w=omega.norm();

  if (w<1.0e-3)
    omega=Vector3(MatrixXd::Zero(3,1));
  else
    omega= omega/w;
  Matrix3 hat_a(hat(omega));
  
  Matrix3 E(MatrixXd::Identity(3,3));
  Matrix3 R;

  R=E + hat_a*sin(w*dt)+ hat_a*hat_a*(1-cos(w*dt));
  return R;
}

bool CalcIVK_biped_toe(const BodyPtr body,const Vector3& CM_p, const Position* pose_ref,
                       const FootType& FT, const string *end_link)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg,SupLeg2W;
  Vector3 cm=body->calcCenterOfMass();
  Vector3  SwLeg_p; 
  Matrix3 W_R,SwLeg_R; 
 
  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  //careful
  MatrixXd Jacobian=MatrixXd::Zero(12,12);//leg only
  int swingLegId;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link("pivot_R");
    swingLegId=0;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link("pivot_L");
    swingLegId=1;
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  SupLeg2SwLeg->calcForwardKinematics();

  //new
  Vector3 SupLeg_p_Init;
  Matrix3 SupLeg_R_Init; 
  SupLeg_p_Init=SupLeg->p();
  SupLeg_R_Init=SupLeg->R();
  
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = 12;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < 12; ++i){
      qorg[i] = body->joint(i)->q();
    }
    
    //careful
    VectorXd dq(n);
    VectorXd v(12);
    double maxIKErrorSqr=1.0e-16;
    
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      W_R=body->link(end_link[WAIST])->R();
      SwLeg_p = SwLeg->p();
      SwLeg_R = SwLeg->R();
      CalJo_biped_toe(body, FT, Jacobian, end_link);        
    
      Vector3 CM_dp=CM_p - body->calcCenterOfMass();// + body->link("LLEG_JOINT5")->p);
      Vector3 W_omega=W_R* omegaFromRot(W_R.transpose() * pose_ref[WAIST].linear());
      Vector3 SW_dp =pose_ref[swingLegId].translation() - SwLeg_p;
      Vector3 SW_omega = SwLeg_R* omegaFromRot(SwLeg_R.transpose() * pose_ref[swingLegId].linear());
      
      //v<< CM_dp, W_omega, SW_dp, SW_omega;
      v.head<3>()=CM_dp;
      v.segment<3>(3)=W_omega;
      v.segment<3>(6)=SW_dp;
      v.segment<3>(9)=SW_omega;

      //double errsqr =  CM_dp.dot(CM_dp) + W_omega.dot(W_omega) + SW_dp.dot(SW_dp) + SW_omega.dot( SW_omega);
      double errsqr=v.squaredNorm();

      if(errsqr < maxIKErrorSqr){
	converged = true;
	break;
      }
      
      MatrixXd JJ;
      double dampingConstantSqr=1e-12;
      JJ = Jacobian *  Jacobian.transpose() + dampingConstantSqr * MatrixXd::Identity(Jacobian.rows(), Jacobian.rows());
      dq = Jacobian.transpose() * QR.compute(JJ).solve(v);


      for(int j=0; j < n; ++j){
	body->joint(j)->q() += LAMBDA * dq(j);
      }
     
      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
      
    }//for
    
    if(!converged){
      
      for(int j=0; j < n; ++j){
	body->joint(j)->q() = qorg[j];
      }
 
      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
    }


    // for(int j=0; j < n; ++j){
    //   ofs2 <<  body->joint(j)->q() << " ";
    // }
    // ofs2 << endl;
    
    
    return converged;          
}


//////ivk_jacobian/////////
void CalJo_biped_toe(const BodyPtr body, const FootType& FT, Eigen::MatrixXd& out_J, const string* end_link)
{ 
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  MatrixXd JSupLeg2SwLeg ,JSupLeg2W,Jcom;

  
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link("pivot_R");
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link("pivot_L");
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  body->calcCenterOfMass();
  calcCMJacobian(body, SupLeg, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
  
  
  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      out_J(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId();
    for(int j=3;j<6;j++){
      out_J(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId();
    for(int j=6;j<12;j++){
      out_J(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
  
}


//////////////for jvrc///////////////////////
bool CalcIVK4Limbs(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, string *end_link)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg,SupLeg2W;
  Vector3 cm=body->calcCenterOfMass();
  Vector3  SwLeg_p; 
  Matrix3 W_R,SwLeg_R; 
  int dof = body->numJoints();

  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  //careful
  MatrixXd Jacobian=MatrixXd::Zero(24,dof);//leg only
  int swingLegId;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
    swingLegId=0;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
    swingLegId=1;
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  //new
  Vector3 SupLeg_p_Init;
  Matrix3 SupLeg_R_Init; 
  SupLeg_p_Init=SupLeg->p();
  SupLeg_R_Init=SupLeg->R();
  
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = dof;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < dof; ++i){
      qorg[i] = body->joint(i)->q();
    }
    
    //careful
    VectorXd dq(n);
    VectorXd v(24);
    double maxIKErrorSqr=1.0e-13;
    
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      W_R=body->link(end_link[WAIST])->R();
      SwLeg_p = SwLeg->p();
      SwLeg_R = SwLeg->R();

      Vector3 rarm_p= body->link(end_link[RARM])->p(); 
      Matrix3 rarm_R= body->link(end_link[RARM])->R(); 

      Vector3 larm_p= body->link(end_link[LARM])->p(); 
      Matrix3 larm_R= body->link(end_link[LARM])->R(); 
      //tag
      CalJo4Limbs(body, FT, Jacobian, end_link);        
      ///
      Vector3 CM_dp=CM_p- body->calcCenterOfMass();// + body->link("LLEG_JOINT5")->p);
      Vector3 W_omega=W_R* omegaFromRot(W_R.transpose() * R_ref[WAIST]);
      Vector3 SW_dp =p_ref[swingLegId] - SwLeg_p;
      Vector3 SW_omega = SwLeg_R* omegaFromRot(SwLeg_R.transpose() * R_ref[swingLegId]);
     
      Vector3 RARM_dp =p_ref[RARM] - rarm_p;  
      Vector3 RARM_omega = rarm_R* omegaFromRot(rarm_R.transpose() * R_ref[RARM]);

      Vector3 LARM_dp =p_ref[LARM] - larm_p;  
      Vector3 LARM_omega = larm_R* omegaFromRot(larm_R.transpose() * R_ref[LARM]); 

      //v<< CM_dp, W_omega, SW_dp, SW_omega;
      v.head<3>()=CM_dp;
      v.segment<3>(3)=W_omega;
      v.segment<3>(6)=SW_dp;
      v.segment<3>(9)=SW_omega;
      v.segment<3>(12)=RARM_dp;
      v.segment<3>(15)=RARM_omega;
      v.segment<3>(18)=LARM_dp;
      v.segment<3>(21)=LARM_omega;

      double errsqr=v.squaredNorm();

      if(errsqr < maxIKErrorSqr){
	converged = true;
	break;
      }
      
      MatrixXd JJ;
      double dampingConstantSqr=1e-12;
      JJ = Jacobian *  Jacobian.transpose() + dampingConstantSqr * MatrixXd::Identity(Jacobian.rows(), Jacobian.rows());
      dq = Jacobian.transpose() * QR.compute(JJ).solve(v);


      for(int j=0; j < n; ++j){
	body->joint(j)->q() += LAMBDA * dq(j);
      }

      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
      
    }//for
    
    if(!converged){
      
      for(int j=0; j < n; ++j){
	body->joint(j)->q() = qorg[j];
      }
 
      SupLeg->p()=SupLeg_p_Init;
      SupLeg->R()=SupLeg_R_Init;
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
    }
    
    return converged;          
}

void CalJo4Limbs(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J, string *end_link)
{ 
  Link* SupLeg;
  Link* SwLeg;
  int dof = body->numJoints();

  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  JointPathPtr  Waist2RARM,Waist2LARM;
  MatrixXd JSupLeg2SwLeg ,JSupLeg2W,Jcom;
  MatrixXd JWaist2RARM, JWaist2LARM;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link(end_link[LLEG]);
    SwLeg=body->link(end_link[RLEG]);
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link(end_link[RLEG]);
    SwLeg=body->link(end_link[LLEG]);
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link(end_link[WAIST]));   

  Waist2RARM = getCustomJointPath(body, body->link("WAIST_R"), body->link(end_link[RARM]));  
  Waist2LARM = getCustomJointPath(body, body->link("WAIST_R"), body->link(end_link[LARM])); 
 
  body->calcCenterOfMass();
  calcCMJacobian(body, SupLeg, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
   
  Waist2RARM->calcJacobian(JWaist2RARM);
  Waist2LARM->calcJacobian(JWaist2LARM);

  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<dof;j++){
      out_J(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W attitute only
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId();
    for(int j=3;j<6;j++){
      out_J(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId();
    for(int j=6;j<12;j++){
      out_J(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
  
 //Waist2RARM
  for(int i = 0; i < Waist2RARM->numJoints(); i++) {
    int id = Waist2RARM->joint(i)->jointId();
    for(int j=12;j<18;j++){
      out_J(j,id)=JWaist2RARM(j-12,i);
    }
  }

  //Waist2LARM
  for(int i = 0; i < Waist2LARM->numJoints(); i++) {
    int id = Waist2LARM->joint(i)->jointId();
    for(int j=18;j<24;j++){
      out_J(j,id)=JWaist2LARM(j-18,i);
    }
  }


}