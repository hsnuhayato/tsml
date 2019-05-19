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
  delete zmpP;
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
  //step_counter = 0;
  //cm_offset_x=0.015;
  coil::stringTo(cm_offset_x, prop["cm_offset_x"].c_str());

  absZMP << m_robot->link(end_link[RLEG])->p()(0) + cm_offset_x, 0.0, 0.0;
  relZMP << m_robot->link(end_link[RLEG])->p()(0) + cm_offset_x, 0.0, 0.0;
  step = 0;
  //cout<<cm_offset_x<<endl;
  flagcalczmp = 0;
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
  coil::stringTo(param.link_b_ankle, prop["link_b_ankle"].c_str());
  coil::stringTo(param.dt, prop["wpg.dt"].c_str());
  coil::stringTo(param.ankle_height, prop["ankle_height"].c_str());

  usePivot = 1;
  stepNum= -1;
  neutralTime = 0;
  freeWalk = 1;

  //test paraini
  velobj = Eigen::MatrixXd::Zero(6,1);
  yawTotal =0;
  object_ref = new Link(); //to be changed to Position
  pt = new Link();
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
  delete pt;
  delete pt_L;
  delete pt_R;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t sony::onExecute(RTC::UniqueId ec_id)
{
  hrp2Base::updateInport();

  //readGamepad();
  // ogawa
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

  //_/_/_/_/_/_/_/_/_/_/_/_/main algorithm_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if (active_control_loop) {
    if (freeWalk) {
      calcRefPoint();
      
      // getnextcom->pop_front CP
      getWalkingMotion();
      calcWholeIVK(); //write in refq
      calcRefZMP();

      // for next step. set p_ref to p_Init
      // if CP empty change leg. calczmpflag =1 here
      // if cp empty and commandIn==5 idle=1
      ifChangeSupLeg();
    }
    // //none freeWalk
    // else {
    //   getWalkingMotion(m_robot, FT, cm_ref, absZMP, p_Init, p_ref, R_ref, rfzmp, zmpP);
    //   ifChangeSupLeg2(m_robot, FT,  zmpP, idle, CommandIn, 
    //                   p_ref, p_Init, R_ref, R_Init, flagcalczmp);
    // }

    //base
    m_basePos.data.x = m_robot->rootLink()->p()(0);
    m_basePos.data.y = m_robot->rootLink()->p()(1);
    m_basePos.data.z = m_robot->rootLink()->p()(2);
    Vector3 rpy = R_ref[WAIST].eulerAngles(2, 1, 0);
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

    // Vector3 tmprf(m_rfsensor.data[0],m_rfsensor.data[1], m_rfsensor.data[2]);
    // Vector3 tmplf(m_lfsensor.data[0],m_lfsensor.data[1], m_lfsensor.data[2]);
    // Vector3 rf_w = m_robot->link(end_link[RLEG])->R() * tmprf;
    // Vector3 lf_w = m_robot->link(end_link[LLEG])->R() * tmplf;
    // ofswu<< rf_w[2] << " " << lf_w[2] <<" " << m_rfsensor.data[2] << 
    //   " " << m_lfsensor.data[2]<<endl;
    // ogawa
    // if( ofs.is_open() ) {
    //   cnoid::Vector3 rpyR, rpyL;
    //   rpyR = R_ref[RLEG].eulerAngles(2,1,0);
    //   rpyL = R_ref[LLEG].eulerAngles(2,1,0);
    //   //ofs << absZMP(0) << " " << absZMP(1) << " " << absZMP(2) << " " << cm_ref(0) << " " << cm_ref(1) << " " << com_ref(2) << std::endl;

    //   ofs << toSec(m_mc.tm);
    //   for(int i=0; i<3; i++)  ofs << " " << absZMP(i);
    //   for(int i=0; i<3; i++)  ofs << " " << cm_ref(i);
    //   for(int i=0; i<3; i++)  ofs << " " << p_ref[RLEG](i);
    //   for(int i=0; i<3; i++)  ofs << " " << p_ref[LLEG](i);
    //   ofs << " " << rpyR(2) << " " << rpyR(1) << " " << rpyR(0);
    //   ofs << " " << rpyL(2) << " " << rpyL(1) << " " << rpyL(0);
    //   ofs << std::endl;
    // }

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

  relZMP = R_ref[WAIST].transpose()*(absZMP - m_robot->link(end_link[WAIST])->p());
  //for(int i=0;i< m_rzmp.data.length();i++)
  //  m_rzmp.data[i]=relZMP[i];    
  m_rzmp.data.x = relZMP[0];
  m_rzmp.data.y = relZMP[1];
  m_rzmp.data.z = relZMP[2];
  m_rzmpOut.write();
}

inline void sony::calcWholeIVK()
{
  
  // ogawa
  if((FT==FSRFsw)||(FT==RFsw)){
    //std::cout << p_ref[RLEG].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    //std::cout << p_ref[LLEG].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  }
  //std::cout << cm_ref.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  

  if(usePivot){
    if(CalcIVK_biped_toe(m_robot, cm_ref, p_ref, R_ref, FT, end_link))
      getIKResult();
    //else
    //cerr<<"ivk err"<<endl;
  }
  else{
    if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, end_link))
      getIKResult();
    else
      cerr<<"ivk err"<<endl;
  }

}

inline void sony::calcRefZMP()
{
  //waiting
  if(idle){
    //NaturalZmp(m_robot, absZMP, cm_offset_x, end_link);
    zmpP->NaturalZmp(m_robot, absZMP, end_link);

  }
  //walking
  else{
    ///rzmp To st
    absZMP[0] = rfzmp.at(0)(0);
    absZMP[1] = rfzmp.at(0)(1);
    absZMP[2] = zmpP->absZMP_z_deque.at(0);
    rfzmp.pop_front();
    zmpP->absZMP_z_deque.pop_front();
  }
}

inline void sony::getIKResult()
{
 getModelPosture(m_robot, m_refq);
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
  object_ref->p() = object_ref->p() + rotRTemp*tmp; // ogawa
}

inline void sony::calcRefFoot()
{
  Matrix3 Rtem_Q=extractYow(object_ref->R());
  //actually in x-y plan only
  RLEG_ref_p = object_ref->p() + Rtem_Q * p_obj2RLEG;
  LLEG_ref_p = object_ref->p() + Rtem_Q * p_obj2LLEG;
  LEG_ref_R= Rtem_Q * R_LEG_ini;
}

inline void sony::prmGenerator()//this is calcrzmp flag
{
  calcRefFoot();
  //////////////usually obmit when keep walking//////////////////
  //start to walk or not
  //waiting
  if (idle) {
    if (step) {
      //start to walk
      pattern = NORMAL;
      //set Initial zmp to zmpP
      resetZmpPlanner();//idle off
     
      //calc trajectory
      gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
      flagcalczmp = 0;
      /*
      std::cout << "sony : ref rfoot pos = "
      << RLEG_ref_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
      std::cout << "sony : ref lfoot pos = "
      << LLEG_ref_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
      std::cout << "sony : p_ref[RLEG] = "
      << p_ref[RLEG].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
      std::cout << "sony : p_ref[LLEG] = "
      << p_ref[LLEG].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
      */
    }
  }
  else if(flagcalczmp == 1){//keep walking start from new leg
    
    if( (!walkJudge(m_robot, FT, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, end_link))&& zmpP->cp_deque.empty() && !step){
      pattern = STOP;
      //cout<<"should stop here"<<endl;
    }
    // if pattern == STOP planstop 
    gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
    flagcalczmp = 0;
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
    /*
      double velsqr=pow(velobj(0),2) + pow(velobj(1),2);
      if( velsqr > 64){
      velobj(0)= velobj(0)* 8/ sqrt(velsqr);
      velobj(1)= velobj(1)* 8/ sqrt(velsqr);
      }
    */
  }
  
  if (m_buttonsIn.isNew()) {
    m_buttonsIn.read();
    //cout<<"buttom is new"<<endl;
    /*
    //wire
    if(m_buttons.data[1]==1)//o button
    step=1;
    else if(m_buttons.data[0]==1)//x burron
    step=0;
    */
  }

}

void sony::resetZmpPlanner()
{// this is for FSRF or FSLF
  Vector3 rzmpInit;
  //NaturalZmp(m_robot, rzmpInit, cm_offset_x, end_link);
  zmpP -> NaturalZmp(m_robot, rzmpInit, end_link);
  zmpP -> setInit(rzmpInit);
  
  idle = 0;//comment out when test mode
}

//TODO:RLEG_ref_p LLEG_ref_p only one
// generate refzmp cm_z cp swingLeg trajectory pivot_b
 void sony::gaitGenerate(FootType FT, Vector3 *p_ref, Matrix3 *R_ref,
                         Vector3 RLEG_ref_p, Vector3 LLEG_ref_p,
                         Matrix3 LEG_ref_R, std::deque<vector2> &rfzmp, ZmpPlaner *zmpP) {
  // Vector3  swLegRef_p;
  // if((FT==FSRFsw)||(FT==RFsw)){
  //   swLegRef_p = RLEG_ref_p;
  // }
  // else if((FT==FSLFsw)||(FT==LFsw)){
  //   swLegRef_p = LLEG_ref_p;
  // }

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  //limit new without p_ref
  Link* SupLeg;
  Vector3 SwLeg_p_ref;
  double limit_y;
  //RLEG_ref_R= LLEG_ref_R=obj
  if((FT==FSRFsw) || FT==RFsw){
    SupLeg = m_robot->link(end_link[LLEG]);
    SwLeg_p_ref = RLEG_ref_p;
    limit_y = -0.17;
  }
  else if((FT==FSLFsw) || FT==LFsw){
    SupLeg=m_robot->link(end_link[RLEG]);
    SwLeg_p_ref = LLEG_ref_p;
    limit_y = 0.17;
  }

  // prevent foot collision
  Vector3 sup_2_sw_leg_abs(SupLeg->R().transpose()*(SwLeg_p_ref - SupLeg->p()));
  if (fabs(sup_2_sw_leg_abs(1)) < 0.17) {
    sup_2_sw_leg_abs(1) = limit_y;
    SwLeg_p_ref = SupLeg->p() + SupLeg->R() * sup_2_sw_leg_abs;
    //adjust
    //swLegRef_p= pfromVector3( SwLeg_p_ref);
    //swLegRef_p = SwLeg_p_ref;
    //cerr<<"interference"<<endl;
  }

  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  
  rfzmp.clear();
  
  if (pattern == NORMAL){
    zmpP -> planCP(m_robot, FT, SwLeg_p_ref, LEG_ref_R, rfzmp, usePivot, end_link);
  } 
  else if (pattern == STOP) {
    //cout<<FT<<" CPstop"<<endl;
    //cout<<"swLegRef_p "<<swLegRef_p<<endl;
    zmpP -> planCPstop(m_robot, FT, p_ref, R_ref, SwLeg_p_ref, LEG_ref_R, rfzmp, end_link);
  }
    
  // else if(CommandIn==2){
  //   //cout<<FT<<" CPstop"<<endl;
  //   //cout<<"swLegRef_p "<<swLegRef_p<<endl;
  //   bool ifLastStep = 1;
  //   zmpP -> planCP(m_robot, FT, SwLeg_p_ref, LEG_ref_R, 
  //     rfzmp, usePivot, end_link, ifLastStep);
  // }

}

// assign all parameter for ik 
void sony::getWalkingMotion()
{
  //capture point 
  zmpP->getNextCom(cm_ref);
  //swingLeg nomal mode
  if (!zmpP->swLegxy.empty()) {
    int swingLeg = swLeg(FT);
    p_ref[swingLeg](0) = zmpP->swLegxy.at(0)[0];
    p_ref[swingLeg](1) = zmpP->swLegxy.at(0)[1];
    //p_ref[swingLeg](2)=p_Init[swingLeg][2]+zmpP->Trajzd.at(0);
    p_ref[swingLeg](2) = zmpP->Trajzd.at(0);
    R_ref[swingLeg] = zmpP->swLeg_R.at(0);
    //zmpP->calcWaistR(FT,  R_ref);
    R_ref[WAIST] = zmpP->calcWaistR(FT, m_robot, end_link);
    cm_ref(2) = zmpP->cm_z_deque.at(0);
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
      T.translation() = Vector3(zmpP->link_b_deque.at(0));
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

      if (fabs(zmpP->rot_pitch.at(0)) < 1e-3) {
        pivot_rot_swLeg = 1;
      }
      R_ref[swingLeg] = zmpP->swLeg_R.at(0) * rotationY(zmpP->rot_pitch.at(0));
      zmpP -> link_b_deque.pop_front();
      zmpP -> rot_pitch.pop_front();
    } 
    ////////////////////////////

    /*
    //contact states ..no good if climb stair
    if(zmpP->Trajzd.at(0)<1e-9)
      m_contactStates.data[swingLeg]=1;
    else 
      m_contactStates.data[swingLeg]=0;
    */

    m_contactStates.data[swingLeg] = zmpP->contactState_deque.at(0);
    if (FT==FSRFsw || FT==LFsw) {
      m_toeheelRatio.data[RLEG] = 1;
      m_toeheelRatio.data[LLEG] = (m_contactStates.data[swingLeg] && pivot_rot_swLeg)?1:0;
    }  else if(FT==FSLFsw || FT==RFsw) {
      m_toeheelRatio.data[LLEG] = 1;
      m_toeheelRatio.data[RLEG] = (m_contactStates.data[swingLeg] && pivot_rot_swLeg)?1:0;
    }
    
    m_controlSwingSupportTime.data[RLEG] = m_controlSwingSupportTime.data[LLEG] = 1;
    if (FT == RFsw || FT == LFsw) {
      m_controlSwingSupportTime.data[swingLeg] = zmpP -> groundAirRemainTime.at(0);
    }

    zmpP -> swLegxy.pop_front();
    zmpP -> Trajzd.pop_front();
    zmpP -> swLeg_R.pop_front();
    zmpP -> cm_z_deque.pop_front();
    zmpP -> contactState_deque.pop_front();
    zmpP -> groundAirRemainTime.pop_front();

      
  }//empty

 
}


void sony::ifChangeSupLeg()
{
  if ((zmpP->cp_deque.empty()) && (!idle)) {
    //cp walking. FSRFsw FSLFsw no foot swing
    if(FT == FSRFsw || FT == LFsw) FT=RFsw;
    else if(FT == FSLFsw || FT==RFsw) FT=LFsw;

    //change leg
    //move updateInit to here
    IniNewStep();
    flagcalczmp = 1;

    prmGenerator();//idle off here
  }
}

// void sony::ifChangeSupLeg2(BodyPtr m_robot, FootType &FT,  ZmpPlaner *zmpP, bool &idle,
//                            int &CommandIn, Vector3 *p_now, Vector3 *p_Init, Matrix3 *R_now, Matrix3 *R_Init, bool &calczmpflag)
// {
//   if(zmpP->cp_deque.empty()){
 
//     if(stepNum>0){
//       //cp walking. FSRFsw FSLFsw no foot swing
//       if(FT==FSRFsw||FT==LFsw)
//         FT=RFsw;
//       else if(FT==FSLFsw||FT==RFsw)
//         FT=LFsw;

//       //change leg
//       IniNewStep(m_robot, FT, zmpP, idle, CommandIn, p_ref, p_Init, R_ref, R_Init);
//       calczmpflag=1;
      
      
//       //LLEG_ref_p[0] = next abs ref pos. deque 
//       //if(stepNum ==2) cp to center
      
//       if(stepNum==1)
//         CommandIn = 5;
//       else if(stepNum == 2)
//         CommandIn = 2;

//       gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
//       stepNum--;
//     }
//     else if(stepNum==0){
//       IniNewStep(m_robot, FT, zmpP, idle, CommandIn, p_ref, p_Init, R_ref, R_Init);
//       stepNum--;//let above execute once
//       cout<<"off active_control_loop"<<endl;
//       neutralTime = 1*200;// (int)(1/dt+NEAR0);
//     }

//     if(neutralTime > 0){
//       neutralTime--;
//       if(neutralTime == 0)
//         active_control_loop = 0;
//     }

//   }

// }
  
void sony::IniNewStep() {
  //copy_poses(pose_init, pose_ref);
  //p_ref >> p_Init
  updateInit(p_ref, p_Init, R_ref, R_Init);
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
  for(int i=0;i<dof;i++) {
    //m_refq.data[i]=body_cur(i)=halfpos[i];
    m_refq.data[i] = body_cur(i) = m_mc.data[i];
  }
  update_model(m_robot, m_mc, FT, end_link);
  
  get_end_link_pose(pose_now, m_robot, end_link);
  //todo to be depricated
  RenewModel(m_robot, p_now, R_now, end_link);

  cm_ref = m_robot -> calcCenterOfMass();// 
  cout<<"sony: cm "<<cm_ref<<endl;
  cout<<"RLEG_p\n "<< m_robot->link(end_link[RLEG])->p()<<endl;
  //cout<<"inipos"<<'\n'<<m_robot->link("RLEG_JOINT5")->R()<<'\n'<<m_robot->link("LLEG_JOINT5")->R()<<endl;
  std::cout << "sony : robot pos = " << m_robot->rootLink()->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;

  //for expos
  copy_poses(pose_init, pose_now);
  copy_poses(pose_ref, pose_now);

  //to be deprecate
  for(int i=0;i<LINKNUM;i++){
    p_Init[i]=p_now[i];
    R_Init[i]=R_now[i];
    p_ref[i]=p_now[i];
    R_ref[i]=R_now[i];
    //p_ref_toe[i]=p_now[i];
    //R_ref_toe[i]=R_now[i];
  }
  //R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);
  R_ref[WAIST] = extractYow(m_robot->rootLink()->R());  // ogawa
  cout << "oga\n" << R_ref[WAIST] << "\n" <<
      m_robot -> rootLink() -> R() << endl;

  // ogawa
  {// TODO: use Rats
    Matrix3 R_R = extractYow(m_robot->link(end_link[RLEG])->R());
    Matrix3 L_R = extractYow(m_robot->link(end_link[LLEG])->R());
    
    Matrix3 Rmid( R_R.transpose() * L_R);
    Vector3 omega( omegaFromRot(Rmid));
    object_ref -> R() = R_R*rodoriges(omega, 0.5);
    object_ref -> p() = (p_Init[RLEG] +  p_Init[LLEG] )/2;
  }


  //class ini
  if( !zmpP ) {
    zmpP= new ZmpPlaner();
    zmpP -> setWpgParam(param);
  }
  //for path planning/////////////////////////////////////////
  //ini
  p_obj2RLEG = object_ref->R().transpose() * (p_Init[RLEG] - object_ref->p()); // ogawa
  p_obj2LLEG = object_ref->R().transpose() * (p_Init[LLEG] - object_ref->p()); // ogawa
  R_LEG_ini = LEG_ref_R= Eigen::MatrixXd::Identity(3,3);

  //object_ref->p()(2)=0;  // comment out by ogawa
  object_ref->p()(2) -= param.ankle_height;  // ogawa
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
    m_robot -> updateLinkTree();
    m_robot -> calcForwardKinematics();

    pose_ref[RLEG] = m_robot -> link("pivot_R") -> T();
    pose_ref[LLEG] = m_robot -> link("pivot_L") -> T();

    //to be deprecated
    p_ref[RLEG] = m_robot -> link("pivot_R") -> p();
    p_ref[LLEG] = m_robot -> link("pivot_L") -> p();
    R_ref[RLEG] = m_robot -> link("pivot_R") -> R();
    R_ref[LLEG] = m_robot -> link("pivot_L") -> R();
  }
  //cout<<m_profile.instance_name<<":pivot "<<m_robot->link("pivot_R")->p()<<endl;
  //cout<<m_robot->link("RLEG_JOINT5")->p()<<endl;
  
  /*  
  pt_R->b()<<0.13, 0.0, -0.105;
  pt_L->b()<<0.13, 0.0, -0.105;
  pt_R->jointType=cnoid::Link::FIXED_JOINT;
  pt_L->jointType=cnoid::Link::FIXED_JOINT;
  m_robot->link("RLEG_JOINT5")->addChild(pt_R);
  m_robot->link("LLEG_JOINT5")->addChild(pt_L);
 
  s2sw_R=JointPathPtr(new JointPath(m_robot->link("LLEG_JOINT5"), m_robot->link("RLEG_JOINT5")->child));
  s2sw_R->calcForwardKinematics();
  s2sw_L=JointPathPtr(new JointPath(m_robot->link("RLEG_JOINT5"), m_robot->link("LLEG_JOINT5")->child));
  s2sw_L->calcForwardKinematics();
  p_ref_toe[RLEG]=m_robot->link("RLEG_JOINT5")->child->p();
  p_ref_toe[LLEG]=m_robot->link("LLEG_JOINT5")->child->p();
  */
  //////////////////////////////////////////////////////////////

  rotRTemp = object_ref->R();
  cout<< "[ " << m_profile.instance_name <<
    "] ref COM_z: " << cm_ref(2) << endl;
  
  //no good when climb stair
  // double w=sqrt(9.806/cm_ref(2));
  // zmpP->setw(w);
  zmpP->setw(cm_ref(2), object_ref->p()(2));  // ogawa
  zmpP->setZmpOffsetX(cm_offset_x);
  Vector3 rzmpInit;
  //NaturalZmp(m_robot, rzmpInit, cm_offset_x, end_link);
  zmpP->NaturalZmp(m_robot, rzmpInit, end_link);

  zmpP->setInit(rzmpInit);//for cp init
  //absZMP(2) = object_ref->p()(2);
  calcRefFoot();

  //ooo
  active_control_loop = 0;

  //ogawa
  absZMP = rzmpInit;
  basePosUpdate();
}

void sony::stepping()
{
  if(freeWalk){

    // ogawa
    if( !active_control_loop ) {
      m_mcIn.read();
      for(int i=0;i<dof;i++) {
        m_refq.data[i]=body_cur(i)=m_mc.data[i];
      }
      update_model(m_robot, m_mc, FT, end_link);
      setCurrentData();
    }


    step =! step;
    cout << "step" << endl;
    active_control_loop=1;
    prmGenerator();//idle off here
    std::cout << "sony : robot pos = " << m_robot->rootLink()->p().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
  }
}

void sony::setFootPosR()
{
  if(freeWalk) {
    freeWalkSwitch();
  }

  // ogawa
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
    
  if(zmpP->cp_deque.empty()){
    FT=FSRFsw;
    //start to walk
    pattern = NORMAL;
    if( idle ){
      std::cout << "setFootPosR : start2walk" << std::endl;
      std::cout << "setFootPosR : stepnum = " << stepNum << std::endl;
      resetZmpPlanner();//idle off
    }
    gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
   
    stepNum = 3;
  }  
  else {
    stepNum+=1;
  }

  active_control_loop=1;
 
  // //start to walk
  // start2walk(m_robot, zmpP, idle);//idle off
  // gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
 
  // stepNum=3; //3 if two steps. number of steps + 1
 
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
  resetZmpPlanner();//idle off
  //use LEG_ref_p as cur status.
  gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
 
  stepNum=3;
  active_control_loop=1;

}

void sony::setFootPosR(double x, double y, double z, double r, double p, double w)
{
  if(freeWalk)
    freeWalkSwitch();


  // ogawa
  if( !active_control_loop ) {
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
  if(zmpP->cp_deque.empty()){
    FT=FSRFsw;
    // start to walk
    pattern = NORMAL;
    if( idle ){
      std::cout << "setFootPosR : start2walk" << std::endl;
      std::cout << "setFootPosR : stepnum = " << stepNum << std::endl;
      resetZmpPlanner();//idle off
    }
    gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
    stepNum = 2;
  }  
  else {
    stepNum+=1;
  }

  active_control_loop=1;
}

void sony::setFootPosL(double x, double y, double z, double r, double p, double w)
{
  LLEG_ref_p[0]=x;
  LLEG_ref_p[1]=y;
  LLEG_ref_p[2]=z;
  LEG_ref_R = cnoid::rotFromRpy(r,p,w);


  // ogawa
  if (!active_control_loop) {
    m_mcIn.read();
    for(int i=0;i<dof;i++) {
      m_refq.data[i]=body_cur(i)=m_mc.data[i];
    }
    update_model(m_robot, m_mc, FT, end_link);
    setCurrentData();
    std::cout << "setFootPosR : set current data" << std::endl;
  }


  
  if (zmpP->cp_deque.empty()) {
    FT = FSLFsw;
    // start to walk
    pattern = NORMAL;
    if (idle) {
      std::cout << "setFootPosL : start2walk" << std::endl;
      std::cout << "setFootPosL : stepnum = " << stepNum << std::endl;
      resetZmpPlanner();//idle off
    }
    gaitGenerate(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
    stepNum = 2;
  }  
  else {
    stepNum+=1;
  }

  active_control_loop=1;
}


void sony::testMove()
{
  //cout<<m_robot->link(end_link[RLEG])->p()<<endl;
  cout<<"test move"<<endl;
  //vector32 zero;
  //zero=MatrixXd::Zero(dof,1);
  Eigen::MatrixXd zero(Eigen::MatrixXd::Zero(dof,1));
  body_cur=MatrixXd::Zero(dof,1);
  m_mcIn.read();
  for(int i=0; i<dof; i++) {
    body_cur(i) = m_mc.data[i];
  }
  /*
  //ver1
  body_ref<<0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,
            0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0,
            0.135465, -0.290561, 0.14261, -1.81385, 1.30413, 0.0651451, 0.202547,  0,
            0.135465, 0.290561, -0.14261, -1.81385, -1.30413, -0.0651451, 0.202547,  0;
  //ver2
 body_ref<<8.40779e-07, 0.00313577, -0.450571, 0.78395, -0.332793, -0.00312566, 
           8.41168e-07, 0.00313571, -0.450557, 0.783916, -0.332772, -0.00312559, 0, 0, 0, 0,
           0.698132, -0.122173, -0, -1.50971, -0.122173, 0, 0, 0, 
           0.698132, 0.122173, 0, -1.50971, 0.122173, 0, 0, 0;
  */

  
  //ver3 cm(0)==0
  /*
  body_ref<<7.6349e-07, 0.00326766, -0.409632, 0.787722, -0.377504, -0.00325755,
    //7.63749e-07, 0.0032676, -0.409578, 0.787609, -0.377443, -0.00325748, 
            7.6349e-07, 0.00326766, -0.409632, 0.787722, -0.377504, -0.00325755,
            0, 0, 0, 0, 
            0.698132, -0.122173, 0, -1.50971, -0.122173, 0, 0, 0,
            0.698132,  0.122173, 0, -1.50971,  0.122173, 0, 0, 0;
  */

  /*
  //ver4 cm(0)==0.015
  body_ref<<7.63538e-07, 0.00326758, -0.376392, 0.784674, -0.407696, -0.00325747,
            7.63538e-07, 0.00326758, -0.376392, 0.784674, -0.407696, -0.00325747, 
           0, 0, 0, 0,
           0.698132, -0.122173, 0, -1.50971, -0.122173, 0, 0, 0,
           0.698132,  0.122173, 0, -1.50971,  0.122173, 0, 0, 0;
  */

  
  body_ref = MatrixXd::Zero(dof,1);
  for(int i=0;i<dof;i++) {
    //m_mc.data[i]=body_ref(i)=halfpos[i];
    body_ref(i)=halfpos[i];
  }
  
  /*
  //for new halfpos
  update_model(m_robot, body_ref, FT, end_link, dof);
  RenewModel(m_robot, p_now, R_now, end_link);
  cm_ref=m_robot->calcCenterOfMass();
 
  body_ref(21) =  deg2rad(-97.2);
  body_ref(34) =  deg2rad(-97.2);
  update_model(m_robot, body_ref, FT, end_link, dof);
  RenewModel(m_robot, p_now, R_now, end_link);

  cout<< R_now[RARM] <<'\n'<<endl;
  cout<< R_now[LARM] <<'\n'<<endl;

  R_now[RARM] = cnoid::rotFromRpy(0, deg2rad(-90),0);
  R_now[LARM] = cnoid::rotFromRpy(0, deg2rad(-90),0);
  */
  /*
  if(CalcIVK4Limbs(m_robot, cm_ref, p_now, R_now, FT, end_link)){
    cout<<"okok"<<endl;
    for(unsigned int i=0;i<dof;i++){
      cout<<m_robot->joint(i)->q()<<",";
    }
    cout<<endl;
  }
  else
    cout<<"no"<<endl;
  */

  /*
  JointPathPtr C2RHAND;
  C2RHAND = getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("R_WRIST_Y"));
  Vector3 RHAND_p =  m_robot->link("R_WRIST_Y")->p();
  Matrix3 RHAND_R = m_robot->link("R_WRIST_Y")->R();
  //RHAND_p(0)+=0.03;
  cout<<"jj "<<C2RHAND->numJoints()<<endl;

  C2RHAND -> setGoal(RHAND_p, RHAND_R);
  if(C2RHAND -> calcInverseKinematics()){
    cout<<"inv OK"<<endl;
  }
  */
  /*
  JointPathPtr C2RHAND;
  C2RHAND=getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("R_WRIST_Y"));
  p_now[RARM]= m_robot->link("R_WRIST_Y")->p();
  p_now[RARM](0) += 0.03;
  R_now[RARM]= m_robot->link("R_WRIST_Y")->R();
  C2RHAND->setGoal(p_now[RARM], R_now[RARM]);
  if(!C2RHAND->calcInverseKinematics()){
    cout<<"inv err"<<endl;
  }
  cout<<"jj "<<C2RHAND->numJoints()<<endl;

  JointPathPtr C2LHAND;
  C2LHAND=getCustomJointPath(m_robot, m_robot->link("WAIST_R"), m_robot->link("L_WRIST_Y"));
  p_now[LARM]= m_robot->link("L_WRIST_Y")->p();
  R_now[LARM]= m_robot->link("L_WRIST_Y")->R();
  C2LHAND->setGoal(p_now[LARM], R_now[LARM]);
  if(!C2LHAND->calcInverseKinematics()){
    cout<<"inv err"<<endl;
  }
  cout<<"jj "<<C2LHAND->numJoints()<<endl;
  */
  /*
  Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 3, bodyDeque_p);
  */


    /*
////////////////////////////////////////
m_robot->calcForwardKinematics();
update_model(m_robot, m_mc, FT, end_link);
RenewModel(m_robot, p_now, R_now, end_link);
cm_ref=m_robot->calcCenterOfMass(); 
//for expos
for(int i=0;i<LINKNUM;i++){
p_ref[i]=p_now[i];
R_ref[i]=R_now[i];
}
R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);
//cm_ref(0)+=0.03;

cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+0.015;
//cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+0.03;  // JVRC

cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+cm_offset_x;

if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, end_link)){
cout<<"okok"<<endl;
for(unsigned int i=0;i<dof;i++){
m_mc.data[i]=body_ref(i)=m_robot->joint(i)->q();
cout<<body_ref(i)<<", ";
}
cout<<endl;
}
else
cout<<"ivk error"<<endl;

  
  
Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 5, bodyDeque);
  //Interplation3(body_cur, zero, body_ref, zero, 5, bodyDeque);
 
  m_robot->calcForwardKinematics();
  update_model(m_robot, m_mc, FT, end_link);
  RenewModel(m_robot, p_now, R_now, end_link);
  //for expos
  for(int i=0;i<LINKNUM;i++){
    p_ref[i]=p_now[i];
    R_ref[i]=R_now[i];
  }
  R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);
  //////////////////////////////////////////////////
  */

  //Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 3, bodyDeque);

  // for(int i=0; i<dof; i++) {
  //   m_mc.data[i] = halfpos[i];
  // }
  // update_model(m_robot, m_mc, FT, end_link);
  // RenewModel(m_robot, p_ref, R_ref, end_link);
  // m_robot->calcForwardKinematics();
  // cm_ref = m_robot->calcCenterOfMass();
  // R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);
  // //cm_ref(0)+=0.03;

  // //cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+0.015;
  // //cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+0.03;  // JVRC

  // cm_ref(0)=m_robot->link(end_link[RLEG])->p()(0)+cm_offset_x;

  // if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, end_link)){
  //   cout<<"okok"<<endl;
  //   for(unsigned int i=0;i<dof;i++){
  //     body_ref(i)=m_robot->joint(i)->q();
  //     cout<<body_ref(i)<<", ";
  //   }
  //   cout<<endl;
  // }
  // else
  //   cout<<"ivk error"<<endl;
  
  
  Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 2, bodyDeque);

  /*
  //
  //new posture
   body_ref<<0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,
            0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0,
       deg2rad(40.0), -deg2rad(7.0),  -deg2rad(0.0),   deg2rad(-86.5),  -deg2rad(7.0), 0.0, 0.0, 0.0,
       deg2rad(40.0),  deg2rad(7.0),   deg2rad(0.0),   deg2rad(-86.5),  deg2rad(7.0), 0.0, 0.0, 0.0 ;
  */

  /*
 for(int i=0;i<32;i++)
   m_mc.data[i]=body_ref(i);
 update_model(m_robot, m_mc, FT);
 RenewModel(m_robot, p_now, R_now);
 cm_ref=m_robot->calcCenterOfMass();// 
 cout<<"cm "<<cm_ref<<endl;
 //for expos
 for(int i=0;i<LINKNUM;i++){
   p_Init[i]=p_now[i];
   R_Init[i]=R_now[i];
   p_ref[i]=p_now[i];
   R_ref[i]=R_now[i];
   p_ref_toe[i]=p_now[i];
   R_ref_toe[i]=R_now[i];
 }
 R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);

 cm_ref(0)=0.015;
 cm_ref(1)=0.0;
 cm_ref(2)=0.827752;
 if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT)){
   cout<<"okok"<<endl;
   for(unsigned int i=0;i<32;i++){
     body_ref(i)=m_robot->joint(i)->q();
     cout<<body_ref(i)<<", ";
   }
   cout<<endl;
 }
 else
   cout<<"ivk error"<<endl;
 Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 3, bodyDeque);
  */

  /*
  cm_ref(0)+=0.125;
  p_ref[RLEG](0)+=0.25;

 if( CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref(i)=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  }
  else 
    cerr<<"errrr"<<endl;
  */

  /*  
  JointPathPtr C2RARM;
  Vector3 tar_p(p_now[RARM]);
  Matrix3 tar_R(R_now[RARM]);
   
  C2RARM=m_robot->getJointPath(m_robot->link("CHEST_JOINT1"), m_robot->link("RARM_JOINT6"));
 pt= new hrp::Link();
  for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  //tar_p(0)+=0.02;
  Vector3 rpyTemp;
  rpyTemp=Vector3(-30*M_PI/180, 0, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * R_now[RARM];

  if( C2RARM->calcInverseKinematics(tar_p, tar_R)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
  }
  else
    cerr<<"inv arm err"<<endl;
  
  SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  */
  
  
  /*
  //pt in aram
  pt->b=0,0.15,0;
  pt->name="sase";
  pt->jointType=hrp::Link::FIXED_JOINT;
  m_robot->link("RARM_JOINT6")->addChild(pt);
  JointPathPtr C2pt;
  //cerr<<m_robot->link("RARM_JOINT6")->child->jointType<<endl;
  //cerr<<m_robot->link("RARM_JOINT6")->d<<endl;

  //cerr<<m_robot->link("sase")->jointType<<endl;//ng
  C2pt=JointPathPtr(new JointPath(m_robot->link("CHEST_JOINT1"), m_robot->link("RARM_JOINT6")->child));
  C2pt->calcForwardKinematics();
  //cerr<< m_robot->link("RARM_JOINT6")->R()<<'\n'<<m_robot->link("RARM_JOINT6")->child->R()<<endl;
  dmatrix gg;
  C2pt->calcJacobian(gg);
  //cerr<<C2pt->numJoints()<<endl;
  
  Vector3  tar_p(m_robot->link("RARM_JOINT6")->child->p());
  Matrix3 tar_R(m_robot->link("RARM_JOINT6")->child->R());
  
   for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  //tar_p(0)+=0.02;
  Vector3 rpyTemp;
  rpyTemp=Vector3(0, 60*M_PI/180, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * m_robot->link("RARM_JOINT6")->child->R();

  if( C2pt->calcInverseKinematics(tar_p, tar_R)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
  }
  else
    cerr<<"inv arm err"<<endl;

    SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  */

  /*
  pt->b=0.13, 0,-0.105;
  //pt->b=0, 0, -0.105;
  pt->name="pivot";
  pt->jointType=hrp::Link::FIXED_JOINT;
  m_robot->link("RLEG_JOINT5")->addChild(pt);

  JointPathPtr s2sw;
  //cerr<<m_robot->link("RARM_JOINT6")->child->jointType<<endl;
  //cerr<<m_robot->link("RARM_JOINT6")->d<<endl;

  //cerr<<m_robot->link("sase")->jointType<<endl;//ng
  s2sw=JointPathPtr(new JointPath(m_robot->link("LLEG_JOINT5"), m_robot->link("RLEG_JOINT5")->child));
  s2sw->calcForwardKinematics();
  cerr<<m_robot->link("RLEG_JOINT5")->child->p()<<'\n'<<m_robot->link("RLEG_JOINT5")->p()<<endl;
  */ 

  /*
  Vector3  tar_p(m_robot->link("RLEG_JOINT5")->child->p());
  Matrix3 tar_R(m_robot->link("RLEG_JOINT5")->child->R());
  for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  Vector3 rpyTemp;
  rpyTemp=Vector3(0, 10*M_PI/180, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * m_robot->link("RLEG_JOINT5")->child->R();
  //tar_p(0)+=0.02;
  //cm_ref(1)=p_ref[LLEG](1);

  p_ref[RLEG]=tar_p;
  R_ref[RLEG]=tar_R;

  if( CalcIVK_biped_toe(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 5);
  }
  else 
    cerr<<"errrr"<<endl;
  */


  while( !bodyDeque.empty() && !active_control_loop ) {
    usleep(10);
  }
  bodyDeque.clear();
  usleep(10);
}

void sony::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{ 
  /*
  vector32 body_cur;
  vector32 body_ref;
  cm_ref(1)=p_ref[LLEG](1);
  for(int i=0;i<m_robot->numJoints();i++)
   body_cur[i]=m_robot->joint(i)->q;

  if( CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 5);
  }
  else 
    cerr<<"errrr"<<endl;
  */

  velobj<< x,y,z,roll,pitch,yaw;
}


// ogawa
void sony::stop()
{
  cout<<"sony write out off"<<endl;
  active_control_loop=0;
}


void sony::freeWalkSwitch()
{
  cout<<"sony write out off"<<endl;
  active_control_loop=0;

  ///////////////////////
  if(!freeWalk){
    // object_ref->p()=(m_robot->link(end_link[RLEG])->p() + m_robot->link(end_link[LLEG])->p())/2;
    // //object_ref->p()(2)=0;
    // object_ref->p()(2) -= param.ankle_height; // ogawa

    // Matrix3 R_R=extractYow(m_robot->link(end_link[RLEG])->R());
    // Matrix3 L_R=extractYow(m_robot->link(end_link[LLEG])->R());
    
    // Matrix3 Rmid( R_R.transpose() * L_R);//for toe
    // Vector3 omega( omegaFromRot(Rmid));
    // object_ref->R()= R_R*rodoriges(omega, 0.5);

    // ogawa
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


//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_

// ogawa
void sony::setCurrentData()
{
  // get_end_link_pose(pose_now, m_robot, end_link);
  // get_end_link_pose(pose_init, m_robot, end_link);
  // get_end_link_pose(pose_ref, m_robot, end_link);
  RenewModel(m_robot, p_now, R_now, end_link);

  cm_ref=m_robot->calcCenterOfMass();
  //for expos
  for(int i=0;i<LINKNUM;i++){
    p_Init[i]=p_now[i];
    R_Init[i]=R_now[i];
    p_ref[i]=p_now[i];
    R_ref[i]=R_now[i];
  }
  R_ref[WAIST]=extractYow(m_robot->rootLink()->R());
 
  object_ref->p()= (p_Init[RLEG] +  p_Init[LLEG] )/2;
  object_ref->p()(2) -= param.ankle_height;
  {
    Matrix3 R_R=extractYow(m_robot->link(end_link[RLEG])->R());
    Matrix3 L_R=extractYow(m_robot->link(end_link[LLEG])->R());
    
    Matrix3 Rmid( R_R.transpose() * L_R);
    Vector3 omega( omegaFromRot(Rmid));
    object_ref->R()= R_R*rodoriges(omega, 0.5);
  }
  rotRTemp=object_ref->R();
  

  if(usePivot){
    p_ref[RLEG]=m_robot->link("pivot_R")->p();
    p_ref[LLEG]=m_robot->link("pivot_L")->p();
    R_ref[RLEG]=m_robot->link("pivot_R")->R();
    R_ref[LLEG]=m_robot->link("pivot_L")->R();
  }

  zmpP->setw(cm_ref(2), object_ref->p()(2));
  Vector3 rzmpInit;
  //NaturalZmp(m_robot, rzmpInit, cm_offset_x, end_link);
  zmpP->NaturalZmp(m_robot, rzmpInit, end_link);
  zmpP->setInit(rzmpInit);//for cp init
  //absZMP(2) = object_ref->p()(2);
  //calcRefFoot();

  absZMP = rzmpInit;
  basePosUpdate();
}


void sony::basePosUpdate()
{
  //base
  m_basePos.data.x=m_robot->rootLink()->p()(0);
  m_basePos.data.y=m_robot->rootLink()->p()(1);
  m_basePos.data.z=m_robot->rootLink()->p()(2);
  Vector3 rpy = R_ref[WAIST].eulerAngles(2, 1, 0);
  //m_baseRpy.data.r=rpy(2);
  //m_baseRpy.data.p=rpy(1);
  m_baseRpy.data.r=0.0;
  m_baseRpy.data.p=0.0;
  m_baseRpy.data.y=rpy(0);
  //ofs<<m_robot->link(end_link[RLEG])->p()(0)<<endl;

  //////////////write///////////////
  rzmp2st();
  m_contactStatesOut.write();
  m_basePosOut.write();
  m_baseRpyOut.write();
}

// void sony::writeOutPortsAll()
// {
//  //////////////write///////////////
//     rzmp2st();
//     m_contactStatesOut.write();
//     m_basePosOut.write();
//     m_baseRpyOut.write();
//     m_lightOut.write();
// }


void sony::logStart(std::string date)
{
  if( !ofs.is_open() ) {
    std::string filepath("/home/player/tsml/log/");
    filepath += (date+"_sony.log");
    ofs.open(filepath.c_str());
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



