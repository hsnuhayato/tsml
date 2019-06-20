// -*- C++ -*-
/*!
 * @file  sony.cpp * @brief sonyComponent * $Date$ 
 *
 * $Id$ 
 */
#include "sony.h"
//std::ofstream ofs("/home/wu/src/HRP3.1x/sony_cnoid/sony.log");
//std::ofstream ofswu("/home/player/tsml/log/sony.log");
// Module specification
// <rtc-template block="module_spec">
static const char* sony_spec[] =
  {
    "implementation_id", "sony",
    "type_name",         "sony",
    "description",       "sonyComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "sonyComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

template <class T> double toSec(T t)
{
  return t.sec + t.nsec / 1000000000.0;
}

sony::sony(RTC::Manager* manager):
  hrp2Base(manager),
  m_axesIn("axes", m_axes),
  m_buttonsIn("buttons", m_buttons),
  m_lightOut("light", m_light),
  m_localEEposOut("localEEpos",m_localEEpos),
  m_sonyServicePort("sonyService")
    // </rtc-template>
{
  m_service0.setComponent(this);
}

sony::~sony()
{
  delete ptnP;
}

RTC::ReturnCode_t sony::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);
  //Set OutPort buffers
  addOutPort("light", m_lightOut);
  addOutPort("localEEpos",m_localEEposOut);
  
  m_light.data.length(1);
  m_light.data[0]=true;
  m_localEEpos.data.length(6);

  RTC::Properties& prop = getProperties();
  coil::stringTo(halfpos, prop["halfpos"].c_str());
  //prop["halfpos"]>>halfpos;

  //for ps3 controller
  m_axes.data.length(29);
  for (int i=0; i<29; i++) {
    m_axes.data[i] = 0;
  }
  m_buttons.data.length(17);
  for (int i=0; i<17; i++) {
    m_buttons.data[i] = 0;
  }
  
  // Set OutPort buffer
  // Set service provider to Ports
  m_sonyServicePort.registerProvider("service0", "sonyService", m_service0);
  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_sonyServicePort);
  //ini base
  hrp2Base::onInitialize();
 
  //pini
  active_control_loop = 0;
  idle = 1;
  coil::stringTo(cm_offset_x, prop["cm_offset_x"].c_str());

  absZMP << m_robot->link(end_link[RLEG])->p()(0) + cm_offset_x, 0.0, 0.0;
  relZMP << m_robot->link(end_link[RLEG])->p()(0) + cm_offset_x, 0.0, 0.0;
  step = 0;
  //cout<<cm_offset_x<<endl;
  FT = FSRFsw;
  pattern = STOP;
  
  //wpgParam
  coil::stringTo(param.Tsup, prop["Tsup"].c_str());
  coil::stringTo(param.Tsup_stepping, prop["Tsup_stepping"].c_str());
  coil::stringTo(param.Tdbl, prop["Tdbl"].c_str());
  coil::stringTo(param.offsetZMPy, prop["offsetZMPy"].c_str());
  coil::stringTo(param.offsetZMPy_stepping, prop["offsetZMPy_stepping"].c_str());
  coil::stringTo(param.offsetZMPx, prop["offsetZMPx"].c_str());
  coil::stringTo(param.Zup, prop["Zup"].c_str());
  coil::stringTo(param.Tv, prop["Tv"].c_str());
  coil::stringTo(param.pitch_angle, prop["pitch_angle"].c_str());
  coil::stringTo(param.link_b_front, prop["link_b_front"].c_str());
  coil::stringTo(param.link_b_rear, prop["link_b_rear"].c_str());
  coil::stringTo(param.link_b_ee, prop["link_b_ee"].c_str());
  coil::stringTo(param.dt, prop["wpg.dt"].c_str());
  coil::stringTo(param.ankle_height, prop["ankle_height"].c_str());

  usePivot = 1;
  stepNum= -1;
  neutralTime = 0;
  freeWalk = 1;
  foot_distance_limit_y = 0.17;
  //test paraini
  velobj = Eigen::MatrixXd::Zero(6,1);
  yawTotal =0;
  object_ref = new Link(); //to be changed to Position
  pt_L = new Link();
  pt_R = new Link();

  //base
  m_basePos.data.x = m_robot->rootLink()->p()(0);
  m_basePos.data.y = m_robot->rootLink()->p()(1);
  m_basePos.data.z = m_robot->rootLink()->p()(2);
  m_baseRpy.data.r = 0.0;
  m_baseRpy.data.p = 0.0;
  m_baseRpy.data.y = 0.0;
  
  cout<<"R\n"<< m_robot->link(end_link[RLEG])->R() << " " <<
    m_robot -> link(end_link[RLEG])->name()<< endl;
  cout<<"L\n"<< m_robot->link(end_link[LLEG])->R()<<endl;
  
  //Link* TLink=forceSensors[0]->link();
  //Link* TLink=m_robot->link("LLEG_JOINT5");
  //for joystick
  buttom_accept = true;

  Eigen::MatrixXd zero(Eigen::MatrixXd::Zero(dof,1));
  body_cur = MatrixXd::Zero(dof,1);
  body_ref = MatrixXd::Zero(dof,1);

  m_mc.data.length(dof);
  for (int i=0; i<dof; i++)
    m_mc.data[i] = 0.0;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t sony::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t sony::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;

  delete object_ref;
  delete pt_L;
  delete pt_R;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t sony::onExecute(RTC::UniqueId ec_id)
{
  hrp2Base::updateInport();

  //readGamepad();
  if (m_basePosInitIn.isNew() && m_baseRpyInitIn.isNew()) {
    m_basePosInitIn.read();
    m_baseRpyInitIn.read();
    //m_mcIn.read();

    if (!active_control_loop) {
      for (int i=0; i<dof; i++) {
        m_robot->joint(i)->q() = m_mc.data[i];
      }
      m_robot->rootLink()->p() << m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z;
      m_robot->rootLink()->R() = cnoid::rotFromRpy(m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y);
      m_robot->calcForwardKinematics();

      setCurrentData();
    }
  }

  //_/_/_/_/_/_/_/main algorithm_/_/_/_/_/_/_/_/_
  if (active_control_loop) {
    if (freeWalk) {
      calcRefPoint();
      // getnextcom->pop_front CP
      getWalkingMotion();
      calcWholeIVK(); //write in refq
      calcRefZMP();

      // for next step. set p_ref to p_Init
      // if cp empty and pattern==STOP idle=1
      ifChangeSupLeg();
    }
    // //TODO: none freeWalk
    // else {
    //   getWalkingMotion(m_robot, FT, cm_ref, absZMP, p_Init, p_ref, R_ref, rfzmp, ptnP);
    //   ifChangeSupLeg2(m_robot, FT,  ptnP, idle, CommandIn, 
    //                   p_ref, p_Init, R_ref, R_Init, flagcalczmp);
    // }

    //base
    m_basePos.data.x = m_robot->rootLink()->p()(0);
    m_basePos.data.y = m_robot->rootLink()->p()(1);
    m_basePos.data.z = m_robot->rootLink()->p()(2);
    Vector3 rpy = pose_ref[WAIST].linear().eulerAngles(2, 1, 0);
    //m_baseRpy.data.r=rpy(2);
    //m_baseRpy.data.p=rpy(1);
    m_baseRpy.data.r = 0.0;
    m_baseRpy.data.p = 0.0;
    m_baseRpy.data.y = rpy(0);
    //ofs<<m_robot->link(end_link[RLEG])->p()(0)<<endl;

    //////////////write///////////////
    rzmp2st();
    m_contactStatesOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_lightOut.write();
    m_toeheelRatioOut.write();
    m_controlSwingSupportTimeOut.write();

    //m_localEEposOut.write();
  }//active_control_loop

  //_/_/_/_/_/_/_/_/_test/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/  
  if(!bodyDeque.empty() && !active_control_loop){
    for(int i=0;i<m_robot->numJoints();i++) {
      m_mc.data[i]=m_refq.data[i]=bodyDeque.at(0)(i);
      //m_refq.data[i]=bodyDeque.at(0)(i);
    }
    bodyDeque.pop_front();
    m_refqOut.write();
  }
    
  return RTC::RTC_OK;
}


//_/_/_/_/_/_/_/_/_function/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_ 
inline void sony::rzmp2st()
{
  //std::cout << "abs 2 rel zmp" << std::endl;
  //std::cout << "abs zmp = " << absZMP.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  // std::cout << "waist p = " << m_robot->link(end_link[WAIST])->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;

  relZMP = pose_ref[WAIST].linear().transpose()*(absZMP - m_robot->link(end_link[WAIST])->p());
  //for(int i=0;i< m_rzmp.data.length();i++)
  //  m_rzmp.data[i]=relZMP[i];    
  m_rzmp.data.x = relZMP[0];
  m_rzmp.data.y = relZMP[1];
  m_rzmp.data.z = relZMP[2];
  m_rzmpOut.write();
}

inline void sony::calcWholeIVK()
{
  if(usePivot){
    if (CalcIVK_biped_ee(m_robot, cm_ref, pose_ref, FT, end_link)) {
      getIKResult();
    }
    //else { cerr<<"ivk err"<<endl;}
  }
  // else{
  //   if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, end_link))
  //     getIKResult();
  //   else
  //     cerr<<"ivk err"<<endl;
  // }

}

inline void sony::calcRefZMP()
{
  //waiting
  if (idle) {
    ptnP -> neutralZmp(m_robot, absZMP, end_link);

  }
  //walking
  else {
    ///rzmp To st
    absZMP = ptnP -> absZMP_deque.at(0);
    ptnP -> absZMP_deque.pop_front();
  }
}

inline void sony::getIKResult()
{
  getModelAngles(m_robot, m_refq);
  m_refqOut.write();
}

inline void sony::calcRefPoint()
{
  //by operator
  Vector3 tmp;
  //translation
  //tep<<velobj(0)*0.00005,velobj(1)*0.00005, velobj(2)*0.00005;
  tmp << velobj(0)*0.00005,velobj(1)*0.00005, 0.0;

  //ref////////
  yawTotal += 0.01 * velobj(5) * M_PI/180;
  Matrix3 rZ(rotationZ( 0.01*velobj(5)*M_PI/180));
  rotRTemp = rZ*rotRTemp ;
  
  object_ref->R() = rotRTemp;
  //object_ref->p() = object_ref->p() + rotationZ(yawTotal)*tmp;
  object_ref->p() = object_ref->p() + rotRTemp*tmp;
}

inline void sony::calcRefFoot()
{
  Matrix3 R = extractYow(object_ref->R());
  //actually in x-y plan only
  RLEG_ref_p = object_ref->p() + R * p_obj2RLEG;
  LLEG_ref_p = object_ref->p() + R * p_obj2LLEG;
  LEG_ref_R = R * R_LEG_ini;
}

inline void sony::ptnGenerator()//this is calcrzmp flag
{
  calcRefFoot();
  //////////////usually obmit when keep walking//////////////////
  //start to walk or not
  //waiting
  if (idle) {
    if (step) {
      //start to walk
      pattern = NORMAL;
      //set Initial zmp to ptnP
      resetPtnPlanner();//idle off
      //calc trajectory
      gaitGenerate();
    }
  }
  else {//keep walking start from new leg
    if ((!walkJudge())&& ptnP->cp_deque.empty() && !step) {
      pattern = STOP;
    }
    // if pattern == STOP planstop 
    gaitGenerate();
  }

}

inline void sony::readGamepad() {
  //gamepad
  if(m_axesIn.isNew()){
    m_axesIn.read();

    velobj(0) = m_axes.data[1] * -20;
    velobj(1) = m_axes.data[0] * -2.5;
    velobj(5) = m_axes.data[2] * -3;

    //wireless
    if (freeWalk) {
      if(m_axes.data[17] >= 0.1) {//o buttom
        step = 1;
        active_control_loop = 1;
      }
      else if(m_axes.data[18] >= 0.1) {//x burrom
        step=0;
      }
    }

    //head
    m_robot->link(HEAD_P)->q()-=0.6*m_axes.data[8]*M_PI/180;
    m_robot->link(HEAD_P)->q()+=0.6*m_axes.data[10]*M_PI/180;
    m_robot->link(HEAD_Y)->q()-=0.6*m_axes.data[9]*M_PI/180;
    m_robot->link(HEAD_Y)->q()+=0.6*m_axes.data[11]*M_PI/180;
    //light
    if(buttom_accept){
      if(m_axes.data[16]>=0.1){//^ buttom
        m_light.data[0]=!m_light.data[0];
        buttom_accept=false;
      }
    }
    else{
      //todo すべてニュートラル状態accept trueにする
      if(m_axes.data[16]==0)
        buttom_accept=true;
    }
  }
  
  if (m_buttonsIn.isNew()) {
    m_buttonsIn.read();
  }

}

inline void sony::resetPtnPlanner()
{// this is for FSRF or FSLF
  Vector3 rzmpInit;
  ptnP -> neutralZmp(m_robot, rzmpInit, end_link);
  ptnP -> setInit(rzmpInit);
  
  idle = 0;
}

//TODO:RLEG_ref_p LLEG_ref_p only one
// generate refzmp cm_z cp swingLeg trajectory pivot_b
inline void sony::gaitGenerate() {
  //limit new without p_ref
  Link* SupLeg = NULL;
  Vector3 SwLeg_p_ref;
  double limit_y;
  //RLEG_ref_R= LLEG_ref_R=obj
  if ((FT==FSRFsw) || FT==RFsw) {
    SupLeg = m_robot->link(end_link[LLEG]);
    SwLeg_p_ref = RLEG_ref_p;
    limit_y = -foot_distance_limit_y;
  }
  else if((FT==FSLFsw) || FT==LFsw) {
    SupLeg = m_robot->link(end_link[RLEG]);
    SwLeg_p_ref = LLEG_ref_p;
    limit_y = foot_distance_limit_y;
  }

  // prevent foot collision
  Vector3 sup_2_sw_leg_abs(SupLeg->R().transpose()*(SwLeg_p_ref - SupLeg->p()));
  if (fabs(sup_2_sw_leg_abs(1)) < foot_distance_limit_y) {
    sup_2_sw_leg_abs(1) = limit_y;
    SwLeg_p_ref = SupLeg->p() + SupLeg->R() * sup_2_sw_leg_abs;
  }

  if (pattern == NORMAL){
    ptnP -> planCP(m_robot, FT, SwLeg_p_ref, LEG_ref_R, usePivot, end_link);
  } 
  else if (pattern == STOP) {
    //cout<<FT<<" CPstop"<<endl;
    //cout<<"swLegRef_p "<<swLegRef_p<<endl;
    ptnP -> planCPstop(m_robot, end_link);
  }
}

// assign all parameter for ik 
inline void sony::getWalkingMotion()
{
  //capture point 
  ptnP -> getNextCom(cm_ref);
  //swingLeg nomal mode
  if (!ptnP->swLeg_xy.empty()) {
    int swingLeg = swLeg(FT);
    pose_ref[swingLeg].translation()(0) = ptnP->swLeg_xy.at(0)[0];
    pose_ref[swingLeg].translation()(1) = ptnP->swLeg_xy.at(0)[1];
    //p_ref[swingLeg](2)=p_Init[swingLeg][2]+ptnP->swLeg_z.at(0);
    pose_ref[swingLeg].translation()(2) = ptnP->swLeg_z.at(0);
    pose_ref[swingLeg].linear() = ptnP->swLeg_R.at(0);
    //ptnP->calcWaistR(FT,  R_ref);
    pose_ref[WAIST].linear() = ptnP->calcWaistR(FT, m_robot, end_link);
    cm_ref(2) = ptnP->cm_z_deque.at(0);
    //cout<<  FT<<" "<< p_ref[swingLeg](2)<<endl;
    /////////toe mode////////////
    bool pivot_rot_swLeg = 0;
    if (usePivot) {
      /*
      for(int i=0;i<3;i++){
      m_localEEpos.data[i]=pivot_localposIni(i);
      m_localEEpos.data[i+3]=pivot_localposIni(i);
      }
      */
      Position T;
      T.linear() = Eigen::MatrixXd::Identity(3,3);
      T.translation() = Vector3(ptnP->link_b_deque.at(0));
      if ((FT==FSRFsw) || (FT==RFsw)) {
        pt_R -> setOffsetPosition(T);

        //for(int i=0;i<3;i++)//right end effect
        //m_localEEpos.data[i]=T.translation()(i);

      }
      else if ((FT==FSLFsw) || (FT==LFsw)) {
        pt_L -> setOffsetPosition(T);
        //for(int i=0;i<3;i++)//left end effect
        //m_localEEpos.data[i+3]=T.translation()(i);
      }

      if (fabs(ptnP->rot_pitch.at(0)) < 1e-3) {
        pivot_rot_swLeg = 1;
      }
      pose_ref[swingLeg].linear() = ptnP->swLeg_R.at(0) * rotationY(ptnP->rot_pitch.at(0));
      ptnP -> link_b_deque.pop_front();
      ptnP -> rot_pitch.pop_front();
    } 
    ////////////////////////////
    m_contactStates.data[swingLeg] = ptnP->contactState_deque.at(0);
    if (FT==FSRFsw || FT==LFsw) {
      m_toeheelRatio.data[RLEG] = 1;
      m_toeheelRatio.data[LLEG] = (m_contactStates.data[swingLeg] && pivot_rot_swLeg)?1:0;
    }  else if(FT==FSLFsw || FT==RFsw) {
      m_toeheelRatio.data[LLEG] = 1;
      m_toeheelRatio.data[RLEG] = (m_contactStates.data[swingLeg] && pivot_rot_swLeg)?1:0;
    }
    
    m_controlSwingSupportTime.data[RLEG] = m_controlSwingSupportTime.data[LLEG] = 1;
    if (FT == RFsw || FT == LFsw) {
      m_controlSwingSupportTime.data[swingLeg] = ptnP -> groundAirRemainTime.at(0);
    }

    ptnP -> swLeg_xy.pop_front();
    ptnP -> swLeg_z.pop_front();
    ptnP -> swLeg_R.pop_front();
    ptnP -> cm_z_deque.pop_front();
    ptnP -> contactState_deque.pop_front();
    ptnP -> groundAirRemainTime.pop_front();
  }//empty

}

inline void sony::ifChangeSupLeg()
{
  if ((ptnP->cp_deque.empty()) && (!idle)) {
    //cp walking. FSRFsw FSLFsw no foot swing
    if(FT == FSRFsw || FT == LFsw) FT=RFsw;
    else if(FT == FSLFsw || FT==RFsw) FT=LFsw;

    //change leg
    //move updateInit to here
    IniNewStep();
    ptnGenerator();//idle off here
  }
}

inline bool sony::walkJudge()
{
  Vector3 FErr_R(VectorXd::Zero(3));
  Vector3 FErr_L(VectorXd::Zero(3));
  Vector3 omega_err_R(VectorXd::Zero(3));
  Vector3 omega_err_L(VectorXd::Zero(3));

  FErr_R =  RLEG_ref_p - m_robot->link(end_link[RLEG])->p();
  FErr_L =  LLEG_ref_p - m_robot->link(end_link[LLEG])->p();
  Matrix3 temR_R = extractYow(m_robot->link(end_link[RLEG])->R());
  Matrix3 temR_L = extractYow(m_robot->link(end_link[LLEG])->R());
  omega_err_R = temR_R * omegaFromRot(temR_R.transpose() * LEG_ref_R);
  omega_err_L = temR_L * omegaFromRot(temR_L.transpose() * LEG_ref_R);

  bool start2walk=0;

  if ((sqrt(FErr_L(0)*FErr_L(0)+FErr_L(1)*FErr_L(1))>0.03)||(sqrt(FErr_R(0)*FErr_R(0)+FErr_R(1)*FErr_R(1))>0.03))
    start2walk=1;//start to walk
  if (omega_err_R.dot(omega_err_R)>0.03||(omega_err_L.dot(omega_err_L)>0.03))
    start2walk=1;//start to walk

  return start2walk;
}

inline void sony::IniNewStep() {
  //ifstop
  if (pattern == STOP) {
    idle = 1;
       
    if ( FT == RFsw)
      FT = FSRFsw;
    else if(FT == LFsw)
      FT = FSLFsw;
  }
}

//_/_/_/_/_/_/_/_/_service port/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_   
void sony::start()//todo change to initialize_wpg
{
  //for expos
  m_mcIn.read();
  //for(unsigned int i=0;i<m_mc.data.length();i++)
  //m_refq.data[i]=body_cur(i)=m_mc.data[i];
  for (int i=0;i<dof;i++) {
    //m_refq.data[i]=body_cur(i)=halfpos[i];
    m_refq.data[i] = body_cur(i) = m_mc.data[i];
  }
  update_model(m_robot, m_mc, FT, end_link);
  get_end_link_pose(pose_now, m_robot, end_link);

  cm_ref = m_robot -> calcCenterOfMass();// 
  cout<<"sony: cm "<<cm_ref<<endl;
  cout<<"RLEG_p\n "<< m_robot->link(end_link[RLEG])->p()<<endl;
  //cout<<"inipos"<<'\n'<<m_robot->link("RLEG_JOINT5")->R()<<'\n'<<m_robot->link("LLEG_JOINT5")->R()<<endl;
  std::cout << "sony : robot pos = " << m_robot->rootLink()->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;

  //for expos
  copy_poses(pose_ref, pose_now);
  pose_ref[WAIST].linear() = extractYow(m_robot->rootLink()->R());

  {// TODO: use Rats
    Matrix3 R_R = extractYow(m_robot->link(end_link[RLEG])->R());
    Matrix3 L_R = extractYow(m_robot->link(end_link[LLEG])->R());
    
    Matrix3 Rmid( R_R.transpose() * L_R);
    Vector3 omega( omegaFromRot(Rmid));
    object_ref -> R() = R_R*rodoriges(omega, 0.5);
    object_ref -> p() = (pose_now[RLEG].translation() +
                         pose_now[LLEG].translation())/2;
  }

  //class ini
  if (!ptnP) {
    ptnP = new patternPlanner();
    ptnP -> setWpgParam(param);
  }
  //for foot planning/////////////////////////////////////////
  //ini
  p_obj2RLEG = object_ref->R().transpose() * (pose_now[RLEG].translation() - object_ref->p());
  p_obj2LLEG = object_ref->R().transpose() * (pose_now[LLEG].translation() - object_ref->p());
  R_LEG_ini = LEG_ref_R= Eigen::MatrixXd::Identity(3,3);

  object_ref->p()(2) -= param.ankle_height;
  std::cout << "[ " << m_profile.instance_name << "] object pos = " <<
    object_ref->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  //for pivot///////////////////////////////////////////////
  if(usePivot){
    Position T;
    T.linear() = Eigen::MatrixXd::Identity(3,3);
    //foot cm offset
    // double ankle_height;
    // RTC::Properties& prop = getProperties();
    // coil::stringTo(ankle_height, prop["ankle_height"].c_str());
    // T.translation() = Vector3(cm_offset_x, 0.0, -ankle_height);
    T.translation() = Vector3(cm_offset_x, 0.0, -param.ankle_height);
    pt_R -> setOffsetPosition(T);
    pt_L -> setOffsetPosition(T);
    pt_R -> setName("pivot_R");
    pt_L -> setName("pivot_L");
    pt_R -> setJointType(cnoid::Link::FIXED_JOINT);
    pt_L -> setJointType(cnoid::Link::FIXED_JOINT);
    m_robot -> link(end_link[RLEG]) -> appendChild(pt_R);
    m_robot -> link(end_link[LLEG]) -> appendChild(pt_L);

    // LinkPtr endLink = m_robot -> link(end_link[RLEG]);
    // while (endLink -> child()) {
    //   endLink = endLink -> child();
    // }
    // endLink -> appendChild(pt_R);

    // endLink = m_robot -> link(end_link[LLEG]);
    // while (endLink -> child()) {
    //   endLink = endLink -> child();
    // }
    // endLink -> appendChild(pt_L);

    m_robot -> updateLinkTree();
    m_robot -> calcForwardKinematics();

    pose_ref[RLEG] = m_robot -> link("pivot_R") -> T();
    pose_ref[LLEG] = m_robot -> link("pivot_L") -> T();
  }
  //cout<<m_profile.instance_name<<":pivot "<<m_robot->link("pivot_R")->p()<<endl;
  //cout<<m_robot->link("RLEG_JOINT5")->p()<<endl;
  //////////////////////////////////////////////////////////////

  rotRTemp = object_ref->R();
  cout<< "[ " << m_profile.instance_name <<
    "] ref COM_z: " << cm_ref(2) << endl;
  
  //no good when climb stair
  // double w=sqrt(9.806/cm_ref(2));
  // ptnP->setw(w);
  ptnP -> setw(cm_ref(2), object_ref->p()(2));
  ptnP -> setZmpOffsetX(cm_offset_x);
  Vector3 rzmpInit;
  ptnP -> neutralZmp(m_robot, rzmpInit, end_link);
  ptnP -> setInit(rzmpInit);//for cp init
  //absZMP(2) = object_ref->p()(2);
  calcRefFoot();
  active_control_loop = 0;

  absZMP = rzmpInit;
  basePosUpdate();
}

void sony::stepping()
{
  if (freeWalk) {
    if (!active_control_loop) {
      m_mcIn.read();
      for (int i=0;i<dof;i++) {
        m_refq.data[i]=body_cur(i)=m_mc.data[i];
      }
      update_model(m_robot, m_mc, FT, end_link);
      setCurrentData();
    }

    step =! step;
    cout << "[ " << m_profile.instance_name << "] step " << step << endl;
    active_control_loop=1;

    if (idle) {
      ptnGenerator();//idle off here
    }

    if (!step) {
      setObjectV(0, 0, 0, 0, 0, 0);
    }

    std::cout << "sony : robot pos = " << m_robot->rootLink()->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  }
}

void sony::setFootPosR()
{
  if(freeWalk) {
    freeWalkSwitch();
  }

  if( !active_control_loop ) {
    m_mcIn.read();
    for(int i=0;i<dof;i++) {
      m_refq.data[i]=body_cur(i)=m_mc.data[i];
    }
    update_model(m_robot, m_mc, FT, end_link);
    setCurrentData();
    std::cout << "setFootPosR : set current data" << std::endl;
  }

  FT=FSRFsw;
  RLEG_ref_p[0]+=0.2;
  LLEG_ref_p[0]+=0.2;
    
  if (ptnP->cp_deque.empty()) {
    FT=FSRFsw;
    //start to walk
    pattern = NORMAL;
    if (idle) {
      std::cout << "setFootPosR : start2walk" << std::endl;
      std::cout << "setFootPosR : stepnum = " << stepNum << std::endl;
      resetPtnPlanner();//idle off
    }
    gaitGenerate();
   
    stepNum = 3;
  }  
  else {
    stepNum+=1;
  }

  active_control_loop=1;
}

void sony::setFootPosL()
{ 
  FT=FSLFsw;
  //LLEG_ref_p[0]+=0.15;
  RLEG_ref_p[0]+=0.35;
  RLEG_ref_p[2]=0;
  LLEG_ref_p[0]+=0.35;
  LLEG_ref_p[2]=0;
  // start to walk
  pattern = NORMAL;
  resetPtnPlanner();//idle off
  //use LEG_ref_p as cur status.
  gaitGenerate();
 
  stepNum=3;
  active_control_loop=1;

}

void sony::setFootPosR(double x, double y, double z, double r, double p, double w)
{
  if (freeWalk) freeWalkSwitch();

  if (!active_control_loop) {
    m_mcIn.read();
    for(int i=0;i<dof;i++) {
      m_refq.data[i]=body_cur(i)=m_mc.data[i];
    }
    update_model(m_robot, m_mc, FT, end_link);
    setCurrentData();
    std::cout << "setFootPosR : set current data" << std::endl;
  }
  RLEG_ref_p[0]=x;
  RLEG_ref_p[1]=y;
  RLEG_ref_p[2]=z;
  LEG_ref_R = cnoid::rotFromRpy(r,p,w);
  /*
  LLEG_ref_p[0]=x;
  LLEG_ref_p[1]=-y;
  LLEG_ref_p[2]=z;
  LEG_ref_R = cnoid::rotFromRpy(r,p,w);
  */
  if (ptnP -> cp_deque.empty()) {
    FT = FSRFsw;
    // start to walk
    pattern = NORMAL;
    if( idle ){
      std::cout << "setFootPosR : start2walk" << std::endl;
      std::cout << "setFootPosR : stepnum = " << stepNum << std::endl;
      resetPtnPlanner();//idle off
    }
    gaitGenerate();
    stepNum = 2;
  }  
  else {
    stepNum+=1;
  }

  active_control_loop=1;
}

void sony::setFootPosL(double x, double y, double z, double r, double p, double w)
{
  LLEG_ref_p[0] = x;
  LLEG_ref_p[1] = y;
  LLEG_ref_p[2] = z;
  LEG_ref_R = cnoid::rotFromRpy(r,p,w);

  if (!active_control_loop) {
    m_mcIn.read();
    for(int i=0;i<dof;i++) {
      m_refq.data[i] = body_cur(i) = m_mc.data[i];
    }
    update_model(m_robot, m_mc, FT, end_link);
    setCurrentData();
    std::cout << "setFootPosR : set current data" << std::endl;
  }

  if (ptnP -> cp_deque.empty()) {
    FT = FSLFsw;
    // start to walk
    pattern = NORMAL;
    if (idle) {
      std::cout << "setFootPosL : start2walk" << std::endl;
      std::cout << "setFootPosL : stepnum = " << stepNum << std::endl;
      resetPtnPlanner();//idle off
    }
    gaitGenerate();
    stepNum = 2;
  }  
  else {
    stepNum += 1;
  }

  active_control_loop = 1;
}

void sony::testMove()
{
  //cout<<m_robot->link(end_link[RLEG])->p()<<endl;
  cout<<"test move"<<endl;
  //vector32 zero;
  //zero=MatrixXd::Zero(dof,1);
  Eigen::MatrixXd zero(Eigen::MatrixXd::Zero(dof,1));
  body_cur = MatrixXd::Zero(dof,1);
  m_mcIn.read();
  for(int i=0; i<dof; i++) {
    body_cur(i) = m_mc.data[i];
  }
  
  body_ref = MatrixXd::Zero(dof,1);
  for(int i=0;i<dof;i++) {
    //m_mc.data[i]=body_ref(i)=halfpos[i];
    body_ref(i)=halfpos[i];
  }
  
  for (int i=0; i<dof; i++) {
    m_mc.data[i] = m_robot->joint(i)->q() = body_ref(i);
  }
  update_model(m_robot, m_mc, FT, end_link);
  get_end_link_pose(pose_now, m_robot, end_link);
  cm_ref = m_robot->calcCenterOfMass(); 
  pose_now[WAIST].linear()=Eigen::MatrixXd::Identity(3,3);
  //cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+0.03;  // JVRC
  cm_ref(0) = m_robot -> link(end_link[RLEG]) -> p()(0) + cm_offset_x;

  if (CalcIVK_biped(m_robot, cm_ref, pose_now, FT, end_link)) {
    for(unsigned int i=0;i<dof;i++){
      body_ref(i) = m_robot->joint(i)->q();
      cout<<body_ref(i)<<", ";
    }
    cout<<endl;
  }
  else
    cout<<"ivk error"<<endl;
  
  Interplation5(bodyDeque, body_cur, zero, zero,
                body_ref, zero, zero, 2.0);

  while (!bodyDeque.empty() && !active_control_loop) {
    usleep(10);
  }
  bodyDeque.clear();
  usleep(10);
}

void sony::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{ 
  velobj<< x,y,z,roll,pitch,yaw;
}

void sony::stop()
{
  cout<<"sony write outport off"<<endl;
  active_control_loop=0;
}

void sony::freeWalkSwitch()
{
  cout<<"sony write out off"<<endl;
  active_control_loop=0;

  ///////////////////////
  if(!freeWalk){
    m_mcIn.read();
    for(int i=0;i<dof;i++) {
      m_refq.data[i]=body_cur(i)=m_mc.data[i];
    }
    update_model(m_robot, m_mc, FT, end_link);
    setCurrentData();
    //object_ref->R()=m_robot->link(end_link[RLEG])->R();
  }

  freeWalk = !freeWalk;
  cout<<"freeWalkMode "<<freeWalk<<endl;
  
}


void sony::freeWalkSwitchOn()
{
  if( !freeWalk )
    freeWalkSwitch();
}


void sony::freeWalkSwitchOff()
{
  if( freeWalk )
    freeWalkSwitch();
}


inline void sony::setCurrentData()
{
  get_end_link_pose(pose_now, m_robot, end_link);
  get_end_link_pose(pose_ref, m_robot, end_link);
  pose_ref[WAIST].linear() =  extractYow(m_robot->rootLink()->R());
  cm_ref = m_robot->calcCenterOfMass();

  object_ref->p() = (pose_now[RLEG].translation() + pose_now[LLEG].translation())/2;
  object_ref->p()(2) -= param.ankle_height;
  {
    Matrix3 R_R=extractYow(m_robot->link(end_link[RLEG])->R());
    Matrix3 L_R=extractYow(m_robot->link(end_link[LLEG])->R());
    
    Matrix3 Rmid( R_R.transpose() * L_R);
    Vector3 omega(omegaFromRot(Rmid));
    object_ref->R() = R_R*rodoriges(omega, 0.5);
  }
  rotRTemp = object_ref->R();
  

  if(usePivot){
    pose_ref[RLEG] = m_robot->link("pivot_R")->T();
    pose_ref[LLEG] = m_robot->link("pivot_L")->T();
  }

  ptnP -> setw(cm_ref(2), object_ref->p()(2));
  Vector3 rzmpInit;
  ptnP -> neutralZmp(m_robot, rzmpInit, end_link);
  ptnP -> setInit(rzmpInit);//for cp init
  //absZMP(2) = object_ref->p()(2);
  //calcRefFoot();

  absZMP = rzmpInit;
  basePosUpdate();
}


inline void sony::basePosUpdate()
{
  //base
  m_basePos.data.x = m_robot->rootLink()->p()(0);
  m_basePos.data.y = m_robot->rootLink()->p()(1);
  m_basePos.data.z = m_robot->rootLink()->p()(2);
  Vector3 rpy = pose_ref[WAIST].linear().eulerAngles(2, 1, 0);
  //m_baseRpy.data.r=rpy(2);
  //m_baseRpy.data.p=rpy(1);
  m_baseRpy.data.r = 0.0;
  m_baseRpy.data.p = 0.0;
  m_baseRpy.data.y = rpy(0);
  //ofs<<m_robot->link(end_link[RLEG])->p()(0)<<endl;

  //////////////write///////////////
  rzmp2st();
  m_contactStatesOut.write();
  m_basePosOut.write();
  m_baseRpyOut.write();
}

void sony::logStart(std::string date)
{
  if( !ofs.is_open() ) {
    std::string filepath("/home/player/tsml/log/");
    filepath += (date+"_sony.log");
    ofs.open(filepath.c_str());
  }
}

int sony::swLeg(const FootType& FT)
{
  //swingLeg
  if ((FT==FSRFsw) || (FT==RFsw)) {
    return RLEG;
  }
  else if ((FT==FSLFsw) || (FT==LFsw)) {
    return LLEG;
  }
}


extern "C"
{
 
  void sonyInit(RTC::Manager* manager)
  {
    coil::Properties profile(sony_spec);
    manager->registerFactory(profile,
                             RTC::Create<sony>,
                             RTC::Delete<sony>);
  }
  
};



