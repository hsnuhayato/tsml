#include<stdio.h>
#include<fstream>
#include<math.h>

#include "patternPlanner.h"

//static std::ofstream ofs("/home/player/tsml/log/zmpp.log");
//static int cnt = 0;

patternPlanner::patternPlanner()
{
  //capture point init
  cp << 0.0, 0.0;
}
 
void patternPlanner::setWpgParam(const wpgParam& param)
{
  Tsup_in = param.Tsup;
  Tsup_stepping_in = param.Tsup_stepping;
  Tp = Tdbl_in = param.Tdbl;
  dt = param.dt;
  offsetZMPy = param.offsetZMPy;
  offsetZMPy_stepping = param.offsetZMPy_stepping;
  offsetZMPx = param.offsetZMPx;
  Zup_in = param.Zup;
  Tv = param.Tv;
  pitch_angle = param.pitch_angle*M_PI/180;
  link_b_front << param.link_b_front[0],param.link_b_front[1],param.link_b_front[2];
  link_b_rear << param.link_b_rear[0],param.link_b_rear[1],param.link_b_rear[2];
  link_b_ee << param.link_b_ee[0],param.link_b_ee[1],param.link_b_ee[2];
  ankle_height = param.ankle_height;

  //new from ankle to zmp
  offsetZMPr = offsetZMPl = link_b_ee;
  offsetZMPr(0) = offsetZMPl(0) = offsetZMPx;
  offsetZMPr(1) = offsetZMPy;
  offsetZMPl(1) = -offsetZMPy;
}
     
void patternPlanner::setInit(const Vector3& Ini)
{
  zmpInit << Ini(0) , Ini(1);
  cp << Ini(0), Ini(1);
}

void patternPlanner::setZmpOffsetX(double &cm_offset_x)
{
  offsetZMPx=cm_offset_x;
  offsetZMPr(0)=offsetZMPl(0)=offsetZMPx;
}
//////////////
void patternPlanner::setw(double &cm_z_in, double groundHeight)
{
  cm_z_cur = cm_z = cm_z_in;
  cm_z -= groundHeight;
  w=sqrt(9.806/cm_z);
  //w= wIn;
}
void patternPlanner::planCP(const BodyPtr m_robot, const FootType& FT, Vector3 swLegRef_p,
                            const Matrix3& footRef_R,
                            const bool usePivot, const string *end_link, const bool& ifLastStep)
{
  absZMP_deque.clear();
  // calc swing foot_xyz R, cm_z pivot_b movement
  planSwingLeg(m_robot, FT, swLegRef_p, footRef_R, usePivot, end_link);

  Vector3 offsetZMP_SupLeg, offsetZMP_SwLeg;
  Link *SupLeg = NULL;
  Link *SwLeg = NULL;

  //for rfzmp
  //////////parameter calculate/////////////////////
  if ((FT==FSRFsw) || (FT==RFsw)) {
    SupLeg = m_robot->link(end_link[LLEG]);
    SwLeg = m_robot->link(end_link[RLEG]);
    
    offsetZMP_SupLeg = offsetZMPl;
    offsetZMP_SwLeg = offsetZMPr;
  }
  else if ((FT==FSLFsw) || (FT==LFsw)) {
    SupLeg = m_robot->link(end_link[RLEG]);
    SwLeg = m_robot->link(end_link[LLEG]);
    
    offsetZMP_SupLeg = offsetZMPr;
    offsetZMP_SwLeg = offsetZMPl;
  }
  Vector3 Sup_cur_p;//maybe call Sup_cur_zmp
  Sup_cur_p = SupLeg->p() + SupLeg->R() * offsetZMP_SupLeg;
 
  Vector3 Sw_cur_p;//maybe call Sw_cur_zmp
  Sw_cur_p = SwLeg->p() + SwLeg->R() * offsetZMP_SwLeg;
 
  offsetZMP_SwLeg(2) = 0;
  swLegRef_p += footRef_R * offsetZMP_SwLeg;
  double abszmp_z_mid = (Sup_cur_p(2) + Sw_cur_p(2))/2;

  ////////////plan//////////////
  Vector2 zero(MatrixXd::Zero(2,1));
  
  if ((FT==FSRFsw)||(FT==FSLFsw)) {
    //for capture point// todo make function below
    Vector2 cp_cur(zmpInit);
    double b = exp(w*Tsup);
    cZMP.head<2>() = (Sup_cur_p.head<2>() - b*cp_cur)/(1-b);
    for(int i=1; i<(int)(Tsup/dt+NEAR0)+1; i++){
      cp = cZMP.head<2>() + exp(w*i*dt) * (cp_cur - cZMP.head<2>());
      cp_deque.push_back(cp);
    }

    //for reference zmp//
    cZMP[2] = abszmp_z_mid;
    extendDeque(absZMP_deque, cZMP, Tsup);
    //Interplation5(cZMP, zero, zero, cZMP, zero, zero,  Tsup, rfzmp);
    //Interplation5(abszmp_z_mid, 0.0, 0.0, abszmp_z_mid, 0.0, 0.0, Tsup, absZMP_z_deque);
  }
  if ((FT==RFsw)||(FT==LFsw)) {

    Vector3 cp_EOF;
    if (!ifLastStep) {
      cp_EOF = swLegRef_p;
    }
    else{
      cp_EOF = (swLegRef_p + Sup_cur_p)/2;
    }
    
    //for capture point//
    extendDeque(cp_deque, cp, Tdbl);
    //vector2 cZMP_pre(cZMP);
    Vector2 cp_cur(cp);
    //double b=exp(w*(Tsup));
    double b = exp(w*(Tsup+Tp));
    //cZMP= (swLegRef_p_v2- b*cp_cur)/(1-b);
    cZMP.head<2>() = (cp_EOF.head<2>() - b*cp_cur)/(1-b);

    //int timeLength=(int)((Tsup)/0.005+NEAR0);
    int timeLength=(int)((Tsup+Tp)/0.005+NEAR0);
    for(int i=1;i<timeLength+1;i++){
      cp = cZMP.head<2>() + exp(w*i*dt) * (cp_cur - cZMP.head<2>());
      cp_deque.push_back(cp);
    }

    static Vector3 v3zero = Vector3d::Zero();
    Interplation5(absZMP_deque, Sw_cur_p, v3zero, v3zero,
                  Sup_cur_p, v3zero, v3zero,
                  2*Tdbl, dt, Tdbl, 2*Tdbl);
    
    cZMP[2] = Sup_cur_p(2);
    extendDeque(absZMP_deque, cZMP, Tsup);

    Interplation5(absZMP_deque, Sup_cur_p, v3zero, v3zero,
                  cp_EOF,  v3zero, v3zero,
                  2*Tp, dt, 0, Tp);
  }

  //cout<<"cp "<<cp_deque.size()<<endl;
}

void patternPlanner::planCPstop(const BodyPtr m_robot, const string *end_link)
{
  absZMP_deque.clear();
  Link* rleg;Link* lleg;
  rleg = m_robot->link(end_link[RLEG]);
  lleg = m_robot->link(end_link[LLEG]);
  Vector3 rleg_cur_p, lleg_cur_p;
  rleg_cur_p = rleg->p() + rleg->R() * offsetZMPr;
  lleg_cur_p = lleg->p() + lleg->R() * offsetZMPl;
  Vector3 mid = (rleg_cur_p + lleg_cur_p)/2;
 
  ////////////plan//////////////
  // quick stop
  for(int i=1;i<(int)( (Tsup+Tdbl) /dt+NEAR0)+1;i++){
    //cp = cZMP+ exp( w*i*dt ) * (cp_cur - cZMP);
    cp = mid.head<2>();
    cp_deque.push_back(cp);
  }
  //for rzmp
  extendDeque(absZMP_deque, mid, Tsup+Tdbl);

  // Interplation5(middle_of_foot, zero, zero, middle_of_foot, zero, zero, Tsup+Tdbl, rfzmp);
  // Interplation5(mid(2), 0.0, 0.0, mid(2), 0.0, 0.0, Tdbl+Tsup, absZMP_z_deque);

}

void patternPlanner::getNextCom(Vector3 &cm_ref)
{
  //if(cp_deque.empty())
  //  return;
  Vector2 cm_cur, cp_cur, cm_vel, cm_out;
  cm_cur<<cm_ref(0), cm_ref(1);

  if(!cp_deque.empty()){
    cp_cur=cp_deque.at(0);
    cp_deque.pop_front();
  }
  else{ 
    cp_cur = cp;
  }
  cm_vel = w * (cp_cur - cm_cur);

  cm_out = cm_cur + cm_vel*dt;
  cm_ref[0] = cm_out[0];
  cm_ref[1] = cm_out[1];

  //ofszmp<<cp_cur[0]<<" "<<cp_cur[1]<<" "<<cm_out[0]<<" "<<cm_out[1]<<endl;
}


Matrix3 patternPlanner::calcWaistR(const FootType& FT, const BodyPtr m_robot, const string *end_link)
{
  Link* SwLeg;
  Link* SupLeg;

  if((FT==FSRFsw)||(FT==RFsw)){
    SwLeg=m_robot->link(end_link[RLEG]);
    SupLeg=m_robot->link(end_link[LLEG]);
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SwLeg=m_robot->link(end_link[LLEG]);
    SupLeg=m_robot->link(end_link[RLEG]);
  } 
  Matrix3 sup_R=extractYow(SupLeg->R());
  Matrix3 sw_R=extractYow(SwLeg->R());

  Matrix3 Rmid( sup_R.transpose() * sw_R);//for toe
  Vector3 omega( omegaFromRot(Rmid));
  //R_ref[WAIST]= sup_R*rodrigues(omega, 0.5);
  Matrix3 R_ref_WAIST= sup_R*rodoriges(omega, 0.5);

  // ofs << cnt << endl;
  // ofs << "sup_R\n" << sup_R <<endl;
  // ofs << "sw_R\n" << sw_R << "\n" <<SwLeg->R() <<endl;
  // ofs << "Rmid\n" << Rmid << endl;
  // ofs << "omega\n" << omega << endl;
  // ofs << "R_waist\n" << R_ref_WAIST << endl;
  // cnt ++;
  return R_ref_WAIST;
}


void patternPlanner::planSwingLeg(const BodyPtr m_robot, const FootType& FT,const Vector3& swLegRef_p,
                                  const Matrix3& tar_R, const bool usePivot, const string *end_link)
{ 
  double zs = 0;
  Vector2 zero(MatrixXd::Zero(2,1));
  Vector3 zerov3(MatrixXd::Zero(3,1));
  swLeg_xy.clear();
  swLeg_z.clear();
  groundAirRemainTime.clear();
  swLeg_R.clear();
  index.clear();

  link_b_deque.clear();
  rot_pitch.clear();

  Link* SwLeg = NULL;
  Link* SupLeg= NULL;
  //Vector3 rpytemp=rpyFromRot(object_ref_R);
  //Matrix3 tar_R=rotationZ(rpytemp(2));
  //Matrix3 tar_R=extractYow(object_ref_R);
  //Matrix3 tar_R=object_ref_R;

  if((FT==FSRFsw)||(FT==RFsw)){
    SwLeg = m_robot->link(end_link[RLEG]);
    SupLeg = m_robot->link(end_link[LLEG]);
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SwLeg = m_robot->link(end_link[LLEG]);
    SupLeg = m_robot->link(end_link[RLEG]);
  }
  
  Matrix3 Rmid(SwLeg->R().transpose() * tar_R);
  Vector3 omega(omegaFromRot(Rmid));

  ///pivot
  Vector3 link_b_s;
  Vector3 link_b_f;
  double pitch_s;
  double pitch_f;

  // calc in swLeg foot coordinate before take off
  Matrix3 swLegIni_R_yaw = extractYow(SwLeg->R());
  Vector3 swLegIni_p_nomal = swLegIni_R_yaw.transpose() * SwLeg->p();
  Vector3 swLegRef_p_nomal = swLegIni_R_yaw.transpose() * swLegRef_p;

  Vector3 swPivotIni_p;
  Vector3 swPivotRef_p;

  //if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0))>0.1){
  if((swLegRef_p_nomal(0) - swLegIni_p_nomal(0)) > 0.05){
    //go forward
    //cout<<"front "<<Tsup<<endl;
    Tsup = Tsup_in;
    Tdbl = Tp=Tdbl_in;
    link_b_s = link_b_front;
    link_b_f = link_b_rear;
    pitch_s = pitch_angle;
    pitch_f = -pitch_angle;
    offsetZMPr(1) =  offsetZMPy;
    offsetZMPl(1) = -offsetZMPy;
  }
  else if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0)) < -0.08){
    //back;
    //cout<<"back "<<endl;
    Tsup = Tsup_in;
    Tdbl = Tp = Tdbl_in;
    link_b_s = link_b_rear;
    link_b_f = link_b_front;
    pitch_s = -pitch_angle*0.5;
    pitch_f =  pitch_angle*0.5;//0.7
     
    offsetZMPr(1) =  offsetZMPy;
    offsetZMPl(1) = -offsetZMPy;
  }
  else{//stepping
    //Tdbl=Tp=0.05;
    Tsup = Tsup_stepping_in;
    Tdbl = Tp = Tdbl_in*0.5;
    link_b_s << offsetZMPx, 0.0, -ankle_height;
    link_b_f << offsetZMPx, 0.0, -ankle_height;
    pitch_s = pitch_f = 0.0;
    offsetZMPr(1) =  offsetZMPy_stepping;
    offsetZMPl(1) = -offsetZMPy_stepping;
  }
    
  // pivot pos
  swPivotIni_p = SwLeg->p() + SwLeg->R()*link_b_s;
  Vector3 link_b_f_tem = link_b_f;
  link_b_f_tem(2) = 0;
  swPivotRef_p = swLegRef_p + tar_R * link_b_f_tem;
  /////////////////////////////////////////////////////
  int TsupNum = (int)((Tsup)/dt + NEAR0);
  int TdblNum = (int)((Tdbl)/dt + NEAR0);
  int TpNum = (int)((Tp)/dt +NEAR0 );

  // if((FT==FSRFsw)||(FT==FSLFsw)) {
  //   //foot no move during this span
  // }
  if((FT==RFsw)||(FT==LFsw)) {
    // pivot x y
    extendDeque(swLeg_xy, (Vector2)swPivotIni_p.head<2>(), Tdbl+Tv);
    double tf = Tsup - 2*Tv;
    minVelInterp x,y;
    x.setParams(swPivotIni_p(0), swPivotRef_p(0), tf);
    y.setParams(swPivotIni_p(1), swPivotRef_p(1), tf);
    int timeLength = (int)(tf/0.005+NEAR0);
    for(int i=1; i < timeLength+1; i++){
      Vector2 temp;
      temp(0) = x.sampling(i*dt);
      temp(1) = y.sampling(i*dt);
      swLeg_xy.push_back(temp);
    }
    extendDeque(swLeg_xy, (Vector2)swPivotRef_p.head<2>(), Tv+Tp);
      
    // pivot z
    double cm_z_tgt;
    {
      zs = 0.0;
      double height=0;
      double lower=0;
      Vector3 SwLegNow_p = SwLeg->p() + SwLeg->R() * link_b_ee;
      Vector3 SupLegNow_p = SupLeg->p() + SupLeg->R() * link_b_ee;
      //double cm_z_tgt = cm_z;
      cm_z_tgt = cm_z_cur;
      double err = 0.001;
      if( SwLegNow_p(2) >swLegRef_p(2)){//downstair
        height = SwLegNow_p(2);
        lower = swLegRef_p(2);

        //if( SwLegNow_p(2) < SupLegNow_p(2))
        cm_z_tgt = lower + cm_z;

      }
      else if( SwLegNow_p(2) <swLegRef_p(2)){//upstair
        height = swLegRef_p(2);
        lower = SwLegNow_p(2);

        if( SwLegNow_p(2) < SupLegNow_p(2) && fabs(SwLegNow_p(2)-SupLegNow_p(2))>err )
          cm_z_tgt = height + cm_z;
      }

      Zup= height + Zup_in;

      extendDeque(swLeg_z, swPivotIni_p(2), Tdbl);
      std::vector<double> X(5), Y(5);
      X[0] = 0.0;
      X[1] = 0.1*Tsup;
      X[2] = 0.5*Tsup;
      X[3] = Tsup-0.011;
      X[4] = Tsup;

      //Y[0]=0.0; Y[1]=Zup_in*0.05;  Y[2]=Zup;  
      //Y[3]=swLegRef_p(2)+Zup_in*0.005;  Y[4]= swLegRef_p(2);   

      Y[0] = swPivotIni_p(2);
      Y[1] = swPivotIni_p(2) + Zup_in * 0.05;
      Y[2] = Zup;
      Y[3] = swPivotRef_p(2) + Zup_in * 0.005;
      Y[4] = swPivotRef_p(2);
  
      tk::spline s;
      s.set_points(X,Y);
      for (int i=0; i<TsupNum; i++) {
        double temz = s((i+1)*dt);
        swLeg_z.push_back(temz);
      }
      extendDeque(swLeg_z, swPivotRef_p(2), Tp);
    }

    // contactState and groundAirRemaintime
    for(int i=0;i<TdblNum;i++) {
      contactState_deque.push_back(1);
      groundAirRemainTime.push_back(Tdbl - (i+1)*dt);
    }
    for(int i=0;i<TsupNum;i++) {
      contactState_deque.push_back(0);
      groundAirRemainTime.push_back(Tsup - (i+1)*dt);

    }
    for(int i=0;i<TpNum;i++) {
      contactState_deque.push_back(1);
      groundAirRemainTime.push_back(Tdbl+Tp - (i+1)*dt);
    }

    //cm_z
    //double cm_z_tgt = lower + cm_z;
    extendDeque(cm_z_deque, cm_z_cur, Tdbl+Tv);
    Interplation3(cm_z_deque, cm_z_cur, 0.0, cm_z_tgt , 0.0, Tsup-2*Tv);
    extendDeque(cm_z_deque, cm_z_tgt, Tv+Tp);
    cm_z_cur = cm_z_tgt;

    // swLeg R
    Interplation5(index, 0.0, 0.0, 0.0 , 1.0, 0.0, 0.0, Tsup-2*Tv);
    int tem = (int)((Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(SwLeg->R());//Tdbl+tv
    while(!index.empty()){
      Matrix3 pushin(SwLeg->R() * rodoriges(omega, index.at(0)));
      swLeg_R.push_back (pushin);//Tsup-2tv
      index.pop_front();
    }
    extendDeque(swLeg_R, tar_R, Tv+Tp);

    // pivot movement
    {
      extendDeque(link_b_deque, link_b_s, Tdbl+Tv);
      minVelInterp x,z;
      double tf = Tsup - 2*Tv;
      x.setParams(link_b_s(0), link_b_f(0), tf);
      z.setParams(link_b_s(2), link_b_f(2), tf);
     
      int timeL = (int)(tf/0.005+NEAR0);
      for(int i=1; i<timeL+1; i++){
        Vector3 temp;
        temp << x.sampling(i*dt), 0.0, z.sampling(i*dt);
        link_b_deque.push_back(temp);
      }
      extendDeque(link_b_deque, link_b_f, Tv+Tp);
    }

    //pitch angle
    {
      std::vector<double> X(5), Y(5);
      X[0] = 0.0;
      X[1] = Tp;
      X[2] = Tp+Tsup;
      X[3] = Tp+Tsup+Tp-0.011;
      X[4] = Tp+Tsup+Tp;
      Y[0] = 0.0;
      Y[1] = pitch_s;
      Y[2] = pitch_f;
      Y[3] = pitch_f*0.005;
      Y[4] = 0.0;
      tk::spline s;
      s.set_points(X,Y);    // currently it is required that X is already sorted
      int timeLength = (int)((Tsup+2*Tp)/dt+NEAR0);
      for(int i=0;i<timeLength;i++){
        rot_pitch.push_back(s((i+1)*dt));
      }
    }

  }
}

//todo use rats
void patternPlanner::neutralZmp(const BodyPtr m_robot, Vector3 &absZMP, const string *end_link)
{
  Vector3 rleg_cur_p, lleg_cur_p;
  rleg_cur_p = m_robot->link(end_link[RLEG])->p() + 
               m_robot->link(end_link[RLEG]) ->R() * offsetZMPr;
  lleg_cur_p = m_robot->link(end_link[LLEG]) ->p() +
               m_robot->link(end_link[LLEG])->R() * offsetZMPl;
  absZMP = (rleg_cur_p + lleg_cur_p)/2;
}
