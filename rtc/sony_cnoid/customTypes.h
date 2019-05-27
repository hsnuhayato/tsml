#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <fstream>
#include <math.h>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/Sensor>
#include <cnoid/Link>
using std::cout; using std::endl;
using std::cerr;
using namespace std;
//using namespace hrp;
using namespace cnoid;
using namespace RTC;
using namespace Eigen;

enum FootType {FSRFsw, FSLFsw, RFsw, LFsw};
enum{RLEG, LLEG, RARM, LARM, WAIST, LINKNUM};
enum StepDir {front,back};

struct wpgParam
{
  double Tsup;
  double Tsup_stepping;
  double Tdbl;
  double dt;
  double offsetZMPy;
  double offsetZMPy_stepping;
  double offsetZMPx;
  double Zup;
  double Tv;
  double pitch_angle;
  std::vector<double> link_b_front;
  std::vector<double> link_b_rear;
  std::vector<double> link_b_ee;
  double ankle_height;
};

//#define Kgain_path "/home/grxuser/users/wu/hrp2rtc/preview_control/prm/"
//#define canonPace 0.09

#ifndef deg2rad
#define deg2rad(x)  (M_PI/180*(x))
#endif
#ifndef rad2deg
#define rad2deg(x)  ((x)*180/M_PI)
#endif

#endif
