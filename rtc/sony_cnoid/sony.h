// -*- C++ -*-
/*!
 * @file  sony.h * @brief sonyComponent * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef SONY_H
#define SONY_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
//add
#include <rtm/CorbaNaming.h>
#include "hrp2Base.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "sonyService_impl.h"
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class sony  : public hrp2Base
{
 public:
  sony(RTC::Manager* manager);
  ~sony();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


  // void ifChangeSupLeg2(BodyPtr m_robot, FootType &FT,  patternPlannery *zmpP, bool &idle, int &CommandIn, Vector3 *p_now, Vector3 *p_Init, Matrix3 *R_now, Matrix3 *R_Init, bool &calczmpflag);

  //service port
  void start();
  void testMove();
  void stepping();
  void setObjectV(double x, double y, double z, double roll, double pitch, double yaw);
  void stop();
  void freeWalkSwitch();
  void freeWalkSwitchOn();
  void freeWalkSwitchOff();

  void setFootPosR();
  void setFootPosL();
  void setFootPosR(double x, double y, double z, double r, double p, double w);
  void setFootPosL(double x, double y, double z, double r, double p, double w);
  void logStart(std::string date);

protected:
  // DataInPort declaration
  RTC::TimedFloatSeq m_axes;
  RTC::InPort<RTC::TimedFloatSeq> m_axesIn;
  RTC::TimedBooleanSeq m_buttons;
  RTC::InPort<RTC::TimedBooleanSeq> m_buttonsIn;
  // DataOutPort declaration
  RTC::TimedBooleanSeq m_light;
  RTC::OutPort<RTC::TimedBooleanSeq> m_lightOut;
  RTC::TimedDoubleSeq m_localEEpos;
  RTC::OutPort<RTC::TimedDoubleSeq> m_localEEposOut;
  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_sonyServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  sonyService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

 private:
  bool active_control_loop;
  bool idle;
  std::vector<double> halfpos;
  //Path planning
  Vector3 p_obj2RLEG, p_obj2LLEG;
  Matrix3 R_LEG_ini;

  Vector3 RLEG_ref_p, LLEG_ref_p;
  Matrix3 LEG_ref_R;
  Vector6 velobj;
  //Position dst;
  Link* object_ref;
  double yawTotal;
  Matrix3 rotRTemp;
  Vector3 pivot_localposIni;

  wpgParam param;
  bool step;
  double foot_distance_limit_y;

  MatrixXd body_ref;
  std::deque<MatrixXd> bodyDeque;

  Link* pt_L;
  Link* pt_R;
  Position ee_offset;
 
  bool usePivot;
  double cm_offset_x;
  //Eigen::MatrixXd gh;
  //for joystick
  bool buttom_accept;
  int stepNum;
  int neutralTime;
  bool freeWalk;

  enum PatternType {NORMAL, STOP} pattern;

  std::ofstream ofs;

  int swLeg(const FootType& FT);

  //function
  inline void rzmp2st();
  inline void calcWholeIVK();
  inline void calcRefZMP();
  inline void getIKResult();
  inline void calcRefPoint();
  inline void calcRefFoot();
  inline void ptnGenerator();
  inline void readGamepad();

  inline void resetPtnPlanner();
  inline void gaitGenerate();
  inline void getWalkingMotion();
  inline void ifChangeSupLeg();
  inline bool walkJudge();
  inline void IniNewStep();

  inline void setCurrentData();
  inline void basePosUpdate();
};


extern "C"
{
  DLL_EXPORT void sonyInit(RTC::Manager* manager);
};

#endif // SONY_H

