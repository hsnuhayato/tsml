#ifndef MYFUNC_H
#define MYFUNC_H 

#define NEAR0 1e-8

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <rtm/CorbaNaming.h>
// OpenHRP
//#include "hrpModel/Body.h"
//#include "hrpModel/Link.h"
//#include "hrpModel/JointPath.h"
//#include "hrpModel/ModelLoaderUtil.h"
//#include "hrpUtil/MatrixSolvers.h"
//#include "hrpUtil/EigenTypes.h" 

#include "customTypes.h"
#include <iostream>
#include <deque>

template<class T>
void extendDeque(std::deque<T>& input, const T val, const double tf, const double dt=0.005)
{
   int num=(int)(tf/dt+NEAR0);
   for (int i=0;i<num;i++) {
     input.push_back(val);
   }
}

template<class T>
void Interplation5(std::deque<T>& input, const T ps, const T dps, const T ddps,
                   const T pf, const T dpf, const T ddpf,
                   const double tf,
                   const double dt=0.005, const double start_time = 0, double end_time = 0)
{
  //cout<<"IP5 template"<<endl;
  T a0,a1,a2,a3,a4,a5;
  if (end_time == 0) {
    end_time = tf;
  }
  int start_idx = (int)(start_time/dt+NEAR0) + 1;
  int end_idx = (int)(end_time/dt+NEAR0) + 1;

  a0=ps;
  a1=dps;
  a2=ddps/2.0;
  a3=(20.0*pf - 20.0*ps - (8.0*dpf + 12.0*dps)*tf - (3.0*ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,3));
  a4=(30.0*ps - 30.0*pf + (14.0*dpf + 16.0*dps)*tf + (3.0*ddps - 2.0*ddpf)*pow(tf,2))/(2.0*pow(tf,4));
  a5=(12.0*pf - 12.0*ps - (6.0*dpf + 6.0*dps)*tf - (ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,5));

  for(int i=start_idx; i<end_idx; i++) {
    double ti=dt*i;
    T tmp;
    tmp=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);
    input.push_back(tmp);
  }
}

template<class T>
void Interplation3(std::deque<T>& input, const T xs, const T dxs,
                   const T xf, const T dxf, const double tf, const double dt=0.005)
{
  T a0,a1,a2,a3;
  int num=(int)(tf/dt+NEAR0);//correct
 
  a0=xs;
  a1=dxs;
  //a2=1/(2*pow(tf,2))*(-6*(xs-xf)+2*(dxs-dxf)*tf);
  //a3=1/pow(tf,3)*(2*(xs-xf)-(dxs-dxf)*tf);
  a2=1/pow(tf,2)*(3*(xf-xs)-(2*dxs+dxf)*tf);
  a3=1/pow(tf,3)*(2*(xs-xf)+(dxs+dxf)*tf);
  
  for(int i=1;i<num+1;i++)//頭抜き
    //for(int i=0;i<num;i++)//ビリ抜き
    {
      double ti=dt*i;
      T tmp;
      tmp=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3);//頭抜き
      input.push_back(tmp);
    }
}

class minVelInterp {
public:
  minVelInterp()  {c << 0,0,0,0;};

  void setParams(const double xs, const double xf, const double tf) {
    Matrix4 E;
    c << xs, xf, 0.0, 0.0;
    E << 1.0, 1.0, 0.0, 1.0,
        exp(tf), exp(-tf), tf, 1.0,
        1.0, -1.0, 1.0, 0.0,
        exp(tf), -exp(-tf), 1.0, 0.0;
    c = E.inverse() * c;
  };

  double sampling(const double dt) {
    return  c(0)*exp(dt) + c(1)*exp(-dt) + c(2)*dt + c(3);
  };

private:
  Vector4 c;
};


////for robot../////
//void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now);

void get_end_link_pose(Position* pose, const BodyPtr body, const string *end_link);

void update_model(const BodyPtr body, const TimedDoubleSeq &m_q, const FootType FT, const string *end_link);

void getModelAngles(const BodyPtr body, TimedDoubleSeq &m_refq);

bool loadtxt(std::string path,  std::deque<double> &data);

Matrix3 extractYow(const Matrix3& Rin);

void copy_poses(Position* pose_copy, const Position* const pose);

bool CalcIVK_biped(const BodyPtr body, const Vector3& CM_p, const Position* pose_ref, const FootType& FT, const string *end_link);

void CalJo_biped(const BodyPtr body, const FootType& FT, Eigen::MatrixXd& out_J, const string *end_link);


bool CalcIVK_biped_ee(const BodyPtr body,const Vector3& CM_p, const Position* pose_ref, const FootType& FT, const string *end_link);
void CalJo_biped_ee(const BodyPtr body, const FootType& FT, Eigen::MatrixXd& out_J, const string *end_link);

//deprecated
bool CalcIVK4Limbs(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, string *end_link);
void CalJo4Limbs(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J, string *end_link);

Matrix3 rodoriges(Vector3 omega, const double dt);

Matrix3 rotationZ(const double theta);

Matrix3 rotationY(const double theta);

#endif
