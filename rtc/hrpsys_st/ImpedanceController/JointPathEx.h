#ifndef __JOINT_PATH_EX_H__
#define __JOINT_PATH_EX_H__
/* #include <hrpModel/Body.h> */
/* #include <hrpModel/Link.h> */
/* #include <hrpModel/JointPath.h> */
/* #include <cmath> */
#include <coil/stringutil.h>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/Sensor>
#include <cnoid/Link>

using namespace cnoid;
// hrplib/hrpUtil/MatrixSolvers.h
namespace hrp {
    int calcSRInverse(const MatrixXd& _a, MatrixXd &_a_sr, double _sr_ratio = 1.0, MatrixXd _w = MatrixXd::Identity(0,0));
};

// hrplib/hrpModel/JointPath.h
namespace hrp {
    class JointPathEx : public JointPath {
  public:
      JointPathEx(BodyPtr& robot, Link* base, Link* end, double control_cycle, bool _use_inside_joint_weight_retrieval = true, const std::string& _debug_print_prefix = "");
    bool calcJacobianInverseNullspace(MatrixXd &J, MatrixXd &Jinv, MatrixXd &Jnull);
    bool calcInverseKinematics2Loop(const Vector3& dp, const Vector3& omega, const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const VectorXd* reference_q = NULL);
    bool calcInverseKinematics2Loop(const Vector3& end_effector_p, const Matrix3& end_effector_R,
                                    const double LAMBDA, const double avoid_gain = 0.0, const double reference_gain = 0.0, const VectorXd* reference_q = NULL,
                                    const double vel_gain = 1.0,
                                    const Vector3& localPos = Vector3d::Zero(), const Matrix3& localR = Matrix3d::Identity());
    bool calcInverseKinematics2(const Vector3& end_p, const Matrix3& end_R, const double avoid_gain = 0.0, const double reference_gain = 0.0, const VectorXd* reference_q = NULL);
    double getSRGain() { return sr_gain; }
    bool setSRGain(double g) { sr_gain = g; return true; }
    double getManipulabilityLimit() { return manipulability_limit; }
    bool setManipulabilityLimit(double l) { manipulability_limit = l; return true; }
    bool setManipulabilityGain(double l) { manipulability_gain = l; return true; }
    void setMaxIKError(double epos, double erot);
    void setMaxIKError(double e);
    void setMaxIKIteration(int iter);
    void setOptionalWeightVector(const std::vector<double>& _opt_w)
    {
        for (unsigned int i = 0 ; i < numJoints(); i++ ) {
            optional_weight_vector[i] = _opt_w[i];
        }
    };
    bool setInterlockingJointPairIndices (const std::vector<std::pair<Link*, Link*> >& pairs, const std::string& print_str = "");
    bool setInterlockingJointPairIndices (const std::vector<std::pair<size_t, size_t> >& pairs);
    void getInterlockingJointPairIndices (std::vector<std::pair<size_t, size_t> >& pairs);
    void getOptionalWeightVector(std::vector<double>& _opt_w)
    {
        for (unsigned int i = 0 ; i < numJoints(); i++ ) {
            _opt_w[i] = optional_weight_vector[i];
        }
    };
  protected:
        double maxIKPosErrorSqr, maxIKRotErrorSqr, maxIKErrorSqr;
        int maxIKIteration;
        bool isBestEffortIKMode;
        std::vector<Link*> joints;
        std::vector<double> avoid_weight_gain, optional_weight_vector;
        // Interlocking joint pairs
        //  pair = [index of joint1, index of joint2], index is considered as index for "joints[index]"
        //  Joint angles of joint1 and joint2 has relathionships.
        //  Currently joint1 = joint2 is assumed.
        std::vector<std::pair<size_t, size_t> > interlocking_joint_pair_indices;
        double sr_gain, manipulability_limit, manipulability_gain, dt;
        std::string debug_print_prefix;
        // Print message Hz management
        std::vector<size_t> joint_limit_debug_print_counts;
        size_t debug_print_freq_count;
        bool use_inside_joint_weight_retrieval;
    };

    typedef boost::shared_ptr<JointPathEx> JointPathExPtr;

    struct VirtualForceSensorParam {
        int id;
        Link* link;
        Vector3 localPos;
        Matrix3 localR;
    };

    void readVirtualForceSensorParamFromProperties (std::map<std::string, hrp::VirtualForceSensorParam>& vfs,
                                                    BodyPtr m_robot,
                                                    const std::string& prop_string,
                                                    const std::string& instance_name);

    void readInterlockingJointsParamFromProperties (std::vector<std::pair<cnoid::Link*, cnoid::Link*> >& pairs,
                                                    BodyPtr m_robot,
                                                    const std::string& prop_string,
                                                    const std::string& instance_name);
};


namespace hrp {
  class InvDynStateBuffer{
    public:
      int N_DOF;
      bool is_initialized;
      double DT;
      VectorXd q, q_old, q_oldold, dq, ddq;
      Vector3 base_p, base_p_old, base_p_oldold, base_v, base_dv;
      Matrix3 base_R, base_R_old, base_dR, base_w_hat;
      Vector3 base_w, base_w_old, base_dw;
      InvDynStateBuffer():is_initialized(false){};
      ~InvDynStateBuffer(){};
      void setInitState(const BodyPtr _m_robot, const double _dt){
        N_DOF = _m_robot->numJoints();
        DT = _dt;
        q.resize(N_DOF);
        q_old.resize(N_DOF);
        q_oldold.resize(N_DOF);
        dq.resize(N_DOF);
        ddq.resize(N_DOF);
        for(int i=0;i<N_DOF;i++)q(i) = _m_robot->joint(i)->q();
        q_oldold = q_old = q;
        dq = ddq = VectorXd::Zero(N_DOF);
        base_p_oldold = base_p_old = base_p = _m_robot->rootLink()->p();
        base_R_old = base_R = _m_robot->rootLink()->R();
        base_dR = base_w_hat = Matrix3d::Zero();
        base_w_old = base_w = base_dw = Vector3d::Zero();
        is_initialized = true;
      };
  };
  // set current Body q,base_p,base_R into InvDynStateBuffer and update vel and acc
  void calcAccelerationsForInverseDynamics(const BodyPtr _m_robot, InvDynStateBuffer& _idsb);
  // set all vel and acc into Body, and call Body::calcInverseDynamics()
  //void calcRootLinkWrenchFromInverseDynamics(BodyPtr _m_robot, InvDynStateBuffer& _idsb, Vector3& _f_ans, Vector3& _t_ans);
  // call calcRootLinkWrenchFromInverseDynamics() and convert f,tau into ZMP
  //void calcWorldZMPFromInverseDynamics(BodyPtr _m_robot, InvDynStateBuffer& _idsb, Vector3& _zmp_ans);
  // increment InvDynStateBuffer for 1 step
  void updateInvDynStateBuffer(InvDynStateBuffer& _idsb);
}

#include <iomanip>

#endif //__JOINT_PATH_EX_H__
