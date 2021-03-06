// -*- C++ -*-
/*!
 * @file  sony.cpp * @brief sonyComponent * $Date$ 
 *
 * $Id$ 
 */
#include "hrp2Base.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrp2Base_spec[] =
  {
    "implementation_id", "hrp2Base",
    "type_name",         "hrp2Base",
    "description",       "hrp2BaseComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "hrp2BaseComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

hrp2Base::hrp2Base(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_qIn("q", m_q),
    m_rhsensorIn("rhsensor", m_rhsensor),
    m_lhsensorIn("lhsensor", m_lhsensor),
    m_rfsensorIn("rfsensor", m_rfsensor),
    m_lfsensorIn("lfsensor", m_lfsensor),
    m_mcIn("mc", m_mc),
    m_rzmpOut("rzmp", m_rzmp),
    m_contactStatesOut("contactStates", m_contactStates),
    m_basePosOut("basePosOut", m_basePos),
    m_baseRpyOut("baseRpyOut", m_baseRpy),
    m_refqOut("refq", m_refq),
    m_basePosInitIn("basePosInit", m_basePosInit),
    m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
    m_toeheelRatioOut("toeheelRatio", m_toeheelRatio),
    m_controlSwingSupportTimeOut("controlSwingSupportTime", m_controlSwingSupportTime),
    ptnP(0)
    //m_hrp2BaseServicePort("hrp2BaseService")
    // </rtc-template>
{
}

hrp2Base::~hrp2Base()
{
}


RTC::ReturnCode_t hrp2Base::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);
  addInPort("rhsensor", m_rhsensorIn);
  addInPort("lhsensor", m_lhsensorIn);
  addInPort("rfsensor", m_rfsensorIn);
  addInPort("lfsensor", m_lfsensorIn);
  addInPort("mc", m_mcIn);

  addInPort("basePosInit", m_basePosInitIn);
  addInPort("baseRpyInit", m_baseRpyInitIn);

  // Set OutPort buffer
  addOutPort("rzmp", m_rzmpOut);
  addOutPort("refq", m_refqOut);
  addOutPort("basePosIn", m_basePosOut);
  addOutPort("baseRpyIn", m_baseRpyOut);
  addOutPort("contactStates", m_contactStatesOut);
  addOutPort("toeheelRatio", m_toeheelRatioOut);
  addOutPort("controlSwingSupportTimeOut", m_controlSwingSupportTimeOut);
  // Set service provider to Ports
  //m_hrp2BaseServicePort.registerProvider("service0", "hrp2BaseService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  //addPort(m_hrp2BaseServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot = bl.load( prop["model"].c_str());
  if (!m_robot) {
    std::cout << m_profile.instance_name << " body load fail " << std::endl;
    return RTC::BAD_PARAMETER;
  }
  dof = m_robot->numJoints();
  std::cout << m_profile.instance_name << " :dof robot " << m_robot -> numJoints() << std::endl;

  std::vector<double> tmp;
  coil::stringTo(tmp, prop["initBasePos"].c_str());
  m_robot -> rootLink() -> p() << tmp[0], tmp[1], tmp[2];
  std::cout << m_profile.instance_name << " :init base pos = " << tmp[0] << ", " << tmp[1] << ", " << tmp[2] << std::endl;
  
  tmp.clear();
  coil::stringTo(tmp, prop["initBaseRpy"].c_str());
  m_robot -> rootLink() -> R() = cnoid::rotFromRpy(tmp[0], tmp[1], tmp[2]);
  std::cout <<  m_profile.instance_name << " :init base rpy = " << tmp[0] << ", " << tmp[1] << ", " << tmp[2] <<"R\n" << m_robot -> rootLink() -> R() << std::endl;


  for(unsigned int i=0;i<dof;i++) {
    m_robot -> joint(i) -> q() = 0; 
  }
  //m_robot->rootLink()->p()(0)=0.0;
  //m_robot->rootLink()->p()=Vector3(0, 0 ,0);
  //std::cout<<"R "<<m_robot->rootLink()->name()<<std::endl;
  m_robot -> calcForwardKinematics();
  Vector3 cm = m_robot -> calcCenterOfMass();
  std::cout<< m_profile.instance_name <<" :centerof mass "<<cm<<endl;//m_robot->mass()

  mass = m_robot->mass();

  end_link[RLEG] = prop["RLEG_END"];
  end_link[LLEG] = prop["LLEG_END"];
  end_link[RARM] = prop["RARM_END"];
  end_link[LARM] = prop["LARM_END"];
  end_link[WAIST] =prop["BASE_LINK"];
  HEAD_P = prop["HEAD_P"];
  HEAD_Y = prop["HEAD_Y"];
  std::cout << m_profile.instance_name << " :rleg end "<< m_robot -> link(end_link[RLEG]) ->p() << std::endl;

  //old for preview control
  //prop["kgain"]>>kgain;
  //prop["fgain"]>>fgain;

  get_end_link_pose(pose_now,  m_robot, end_link);
  //////////
  m_robot -> calcCenterOfMass();

  //sensor set
  //fsensorRLEG = m_robot->sensor<cnoid::ForceSensor>(0);
  //fsensorLLEG = m_robot->sensor<cnoid::ForceSensor>(1);
  forceSensors = m_robot->devices();
  AccelSensors = m_robot->devices();
  RateGyroSensors = m_robot->devices();
  //cout<<"forcesener localp "<<'\n'<<forceSensors[0]->p_local()<<endl;
  //cout<<"forcesener absp "<<'\n'<<m_robot->link(end_link[RLEG])->p() + m_robot->link(end_link[RLEG])->R() * forceSensors[0]->p_local()<<endl;
  //cout<<"forcesener localR "<<'\n'<<forceSensors[0]->R_local()<<endl;
  //for(int i=0;i< (m_robot->numDevices());i++)
  cout<<"device num "<<m_robot->numDevices()<<endl;
  //cout<<"name "<<forceSensors[0]->name()<<endl;
  //cout<<"name "<<forceSensors[1]->name()<<endl;
  cout<<"name "<<AccelSensors[0]->name()<<endl;
  //Link* TLink=forceSensors[0]->link();
  //Link* TLink=m_robot->link("LLEG_JOINT5");
  RateGyroSensor* sen = RateGyroSensors[0];
  cout << "Gyro in link: "<< sen->link()->name() << endl;

  // Isometry3 local;
  // local.linear() = AccelSensors[0] -> R_local();
  // local.translation() = AccelSensors[0] -> p_local();
  // cout<<  local.translation()<<'\n'<<  local.linear()<<endl;
  // cout<<"forcesener f "<<'\n'<<forceSensors[0]->f()<<endl;
  // cout<<"forcesener tau "<<'\n'<<forceSensors[0]->tau()<<endl;
  // Matrix3 testR= forceSensors[0]->link()->R()* forceSensors[0]->R_local();
  ForceSensor* sensor = forceSensors[0];
  Matrix3 testR = sensor->link()->R() * sensor->R_local();
  Vector3 sep = sensor->link()->p() + testR * sensor->p_local();
  cout<<"test R"<<'\n'<< sensor->R_local() << '\n' << sensor->p_local() <<endl;
  cout<<sensor->link()->name()<<endl;

  ForceSensor* sensor1 = forceSensors[1];
  testR=sensor1->link()->R()* sensor1->R_local();
  sep = sensor1->link()->p() + testR * sensor1->p_local();
  cout<<"test R2"<<'\n'<< sensor1->R_local()<< '\n' << sensor1->p_local() <<endl;
  cout<<sensor1->link()->name()<<endl;

  //data port
  m_q.data.length(dof);
  m_mc.data.length(dof);
  m_rhsensor.data.length(6);
  m_lhsensor.data.length(6);
  m_rfsensor.data.length(6);
  m_lfsensor.data.length(6);  
  //m_rzmp.data.length(3); 
  m_refq.data.length(dof);
  m_contactStates.data.length(4);
  m_contactStates.data[0]=m_contactStates.data[1]=1;
  m_contactStates.data[2]=m_contactStates.data[3]=0;

  m_toeheelRatio.data.length(4);
  m_toeheelRatio.data[0]=m_toeheelRatio.data[1]=1;
  m_toeheelRatio.data[2]=m_toeheelRatio.data[3]=1;
  
  m_controlSwingSupportTime.data.length(4);
  m_controlSwingSupportTime.data[0]=m_controlSwingSupportTime.data[1]=1;
  m_controlSwingSupportTime.data[2]=m_controlSwingSupportTime.data[3]=1;
  return RTC::RTC_OK;
}

void hrp2Base::updateInport()
{
  if(m_mcIn.isNew()){
    m_mcIn.read();
  }
  if( m_rhsensorIn.isNew() ) 
    m_rhsensorIn.read();
  if( m_lhsensorIn.isNew() ) 
    m_lhsensorIn.read();
  if( m_rfsensorIn.isNew() ) 
    m_rfsensorIn.read();
  if( m_lfsensorIn.isNew() ) 
    m_lfsensorIn.read();
}


extern "C"
{
 
  void hrp2BaseInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrp2Base_spec);
    manager->registerFactory(profile,
                             RTC::Create<hrp2Base>,
                             RTC::Delete<hrp2Base>);
  }
  
};
 


