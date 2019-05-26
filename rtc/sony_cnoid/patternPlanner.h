#ifndef ZMPPLANER_H
#define ZMPPLANER_H

#include <deque>

//#define NEAR0 1e-8

#include "commonFunc.h"
#include "spline.h"
#include "customTypes.h"
//use preview control operater
//#include "preview_control/PreviewControl.h"
class patternPlanner {
  
 public:
  patternPlanner();
  //patternPlanner(FootType FT, double *prm);
  ~patternPlanner();
  void setInit(const Vector3& Ini);
  //void calcWaistR( FootType FT,  Matrix3 *R_ref);
  Matrix3 calcWaistR(const FootType& FT, const BodyPtr m_robot, const string *end_link);

  //capture point/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  void planCP(const BodyPtr m_robot, const FootType& FT, Vector3 swLegRef_p, const Matrix3& footRef_R,  const bool usePivot, const string *end_link, const bool& ifLastStep = 0);

  void planCPstop(const BodyPtr m_robot, const string *end_link);
  // plan swing leg and COM in z direction
  void planSwingLeg(const BodyPtr m_robot, const FootType& FT,const Vector3& swLegRef_p, const Matrix3& tar_R,
                      const bool usePivot, const string *end_link);

  void setw(double &cm_z_in, double groundHeight=0.0);  // ogawa
  void setZmpOffsetX(double &cm_offset_x);

  void getNextCom(Vector3 &cm_ref);
  void setWpgParam(const wpgParam& param);
  void neutralZmp(const BodyPtr m_robot, Vector3 &absZMP,const string *end_link);

  std::deque<Vector3> swingLegTraj;//x y theta
  std::deque<vector2> swLeg_xy;
  std::deque<double> swLeg_z;
  std::deque<double> groundAirRemainTime;
  std::deque<Matrix3> swLeg_R;
  //std::deque<Matrix3> rot_pitch;
  std::deque<double> rot_pitch;
  std::deque<double> index;
  //for pitch
  std::deque<Vector3> link_b_deque;
  Vector3 link_b_front;
  Vector3 link_b_rear;
  Vector3 link_b_ee;

  double offsetZMPx;
  double offsetZMPy;
  double offsetZMPy_stepping;
 
  int beforeUpNum;
  double Tsup;
  double Tsup_in;
  double Tsup_stepping_in;
  double Tdbl;
  double Tdbl_in;
  double ankle_height;

  deque<double> cm_z_deque;
  deque<Vector3> absZMP_deque;
  deque<bool> contactState_deque;
  //for capture point
  std::deque<vector2> cp_deque;
  vector2 cp;//last cp of one step
  double w;
  Vector3 cZMP;
  vector2 cm_vel;
  std::deque<vector2> toe_heel_ratio;
  ///

 private:
  //new
  vector2 zmpInit;
  Vector3 offsetZMPr;
  Vector3 offsetZMPl;

  double Zup;
  double Zup_in;
  double Tv;
  double dt;
  int it;
  double *zmptemp;
  double pitch_angle;
  double Tp;
  
  double cm_z;
  double cm_z_cur;
  
  double stopPoint;
  double pitchMax;

};

#endif
