#include "gtest/gtest.h"
#include "commonFunc.h"


class sonyTest : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    BodyLoader bl;
    m_robot = bl.load("../../../model/JVRC-TSML/main_bush.wrl");  
    m_robot->rootLink()->p() << 0.0,0.0,0.846;
    end_link[RLEG] = "R_ANKLE_P";
    end_link[LLEG] = "L_ANKLE_P";
    end_link[RARM] = "R_WRIST_P";
    end_link[LARM] = "L_WRIST_P";
    end_link[WAIST] = "PELVIS";

    for (int i=0; i<m_robot->numJoints(); i++) {
      m_robot -> joint(i) -> q() = 0.0;
    }
    m_robot -> calcForwardKinematics();
    
  }

  // member
  static BodyPtr m_robot;
  static string end_link[LIMBNUM];
};

BodyPtr sonyTest::m_robot;
string sonyTest::end_link[LIMBNUM];

TEST_F(sonyTest, modelTest) {
  ASSERT_TRUE(m_robot);
  cout << m_robot->numJoints() << endl;

  double q[] = {-0.472363, -8.68958e-18, 8.5109e-18, 1.01319, 5.09607e-18, -0.540829, -0.472363, -8.76276e-18, 7.16418e-18, 1.01319, 5.85352e-18, -0.540829, 0, 0, 0, 0, 0, 0, 0.174533, -0.0872665, 0, -0.523599, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.174533, 0.0872665, 0, -0.523599, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (int i=0; i<m_robot->numJoints(); i++) {
    m_robot -> joint(i) -> q() = q[i];
  }
  
  JointPathPtr Rf2Lf = getCustomJointPath(m_robot, m_robot->link(end_link[RLEG]), m_robot->link(end_link[LLEG]));
  Rf2Lf -> calcForwardKinematics();
  m_robot -> calcForwardKinematics();
  Vector3 cm = m_robot -> calcCenterOfMass();
  cout << m_robot -> link("R_ANKLE_P") -> p() << endl;
  cout << m_robot -> link("L_ANKLE_P") -> p() << endl;

  Vector3 bush_height = Vector3d::Zero();
  LinkPtr endLink = m_robot -> link(end_link[RLEG]);
  while (endLink -> child()) {
    endLink = endLink -> child();
    bush_height += endLink->b();
  }
  Position T;
  Link* pt_R;
  Link* pt_L;
  pt_L = new Link();
  pt_R = new Link();
  
  T.linear() = Eigen::MatrixXd::Identity(3,3);
  T.translation() = Vector3(0.015, 0.0, -0.1-bush_height[2]);
  pt_R -> setOffsetPosition(T);
  pt_L -> setOffsetPosition(T);
  pt_R -> setName("pivot_R");
  pt_L -> setName("pivot_L");
  pt_R -> setJointType(cnoid::Link::FIXED_JOINT);
  pt_L -> setJointType(cnoid::Link::FIXED_JOINT);
    
  endLink -> appendChild(pt_R);

  endLink = m_robot -> link(end_link[LLEG]);
  while (endLink -> child()) {
    endLink = endLink -> child();
  }
  endLink -> appendChild(pt_L);

  m_robot -> updateLinkTree();
  m_robot -> calcForwardKinematics();

  cout <<"pr\n" << m_robot -> link("pivot_R") -> p() << endl;
  cout <<"pl\n" << m_robot -> link("pivot_L") -> p() << endl;

  JointPathPtr Lf2R = getCustomJointPath(m_robot, m_robot->link(end_link[RLEG]), m_robot->link(end_link[LLEG]));
  FootType FT = RFsw;
  Vector3 cm_ref = m_robot->calcCenterOfMass();
  Position pose_ref[LIMBNUM];
  get_end_link_pose(pose_ref, m_robot, end_link);
  pose_ref[RLEG] = m_robot -> link("pivot_R") -> T();
  pose_ref[LLEG] = m_robot -> link("pivot_R") -> T();
  pose_ref[RLEG].translation()[2] += 0.01;
  if (CalcIVK_biped_ee(m_robot, cm_ref, pose_ref, FT, end_link)) {
    cout <<"pr\n" << m_robot -> link("pivot_R") -> p() << endl;
    cout <<"pl\n" << m_robot -> link("pivot_L") -> p() << endl;
    
  } else {
    EXPECT_TRUE(false);
  }
  
  
  EXPECT_TRUE(true);
}