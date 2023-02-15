#ifndef __VELMA_STATE_VALIDATOR__H__
#define __VELMA_STATE_VALIDATOR__H__

#include <array>
#include <vector>

#include <kdl/frames.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

// plugin for robot interface
#include <pluginlib/class_loader.h>
#include <rcprg_planner/robot_interface.h>

#include <planer_utils/double_joint_collision_checker.h>

typedef std::array<std::vector<double>, 7 > ArmLimits;

class VelmaStateValidator;
typedef std::shared_ptr<VelmaStateValidator > VelmaStateValidatorPtr;

class VelmaStateValidatorLoader {
protected:
  pluginlib::ClassLoader<rcprg_planner::RobotInterface> m_robot_interface_loader;
  std::string m_robot_interface_plugin_str;
  std::string m_robot_description_str;
  std::string m_robot_description_semantic_str;

  ArmLimits m_right_arm_limits;
  ArmLimits m_left_arm_limits;

  double m_torso_limit_lo;
  double m_torso_limit_up;

  std::vector<double > m_wcc_l_constraint_polygon;
  int m_wcc_l_joint0_idx;
  int m_wcc_l_joint1_idx;
  double m_wcc_r_d0;
  double m_wcc_l_d0;

  std::vector<double > m_wcc_r_constraint_polygon;
  int m_wcc_r_joint0_idx;
  int m_wcc_r_joint1_idx;

  void readJointLimits(ros::NodeHandle& nh);

public:
  VelmaStateValidatorLoader(ros::NodeHandle& nh);

  VelmaStateValidatorPtr create();
};

class VelmaStateValidator {
public:
  friend class VelmaStateValidatorLoader;

  enum ArmSide {ARM_R=0, ARM_L=1};
  typedef std::array<double, 7> ArmJntArray;

protected:
  std::vector<planning_scene::PlanningScenePtr > m_planning_scenes;
  robot_model::RobotModelPtr m_robot_model;
  std::shared_ptr<moveit::core::RobotState > m_ss;
  boost::shared_ptr<rcprg_planner::RobotInterface> m_robot_interface;
  bool m_verbose;

  ArmLimits m_right_arm_limits;
  ArmLimits m_left_arm_limits;

  double m_torso_limit_lo;
  double m_torso_limit_up;

  static const std::array<std::string, 7 > m_right_arm_joint_names;
  static const std::array<std::string, 7 > m_left_arm_joint_names;

  static const std::array<std::string, 8 > m_right_hand_joint_names;
  static const std::array<std::string, 8 > m_left_hand_joint_names;

  // ROS parameters
  // std::vector<double > wcc_l_constraint_polygon_;
  // int wcc_l_joint0_idx_;
  // int wcc_l_joint1_idx_;
  // double wcc_r_d0_;
  // double wcc_l_d0_;

  // std::vector<double > wcc_r_constraint_polygon_;
  // int wcc_r_joint0_idx_;
  // int wcc_r_joint1_idx_;

  std::shared_ptr<DoubleJointCC > m_wcc_l;
  std::shared_ptr<DoubleJointCC > m_wcc_r;

  void readJointLimits(ros::NodeHandle& nh);

  bool isArmInLimits(const ArmJntArray& q, const ArmLimits& arm_limits) const;

  VelmaStateValidator(boost::shared_ptr<rcprg_planner::RobotInterface> robot_interface,
                                                        robot_model::RobotModelPtr m_robot_model,
                                                        std::shared_ptr<DoubleJointCC > wcc_l,
                                                        std::shared_ptr<DoubleJointCC > wcc_r);
public:

  //VelmaStateValidator(ros::NodeHandle& nh);

  void setVerbose(bool verbose);

  void setVariablePosition(const std::string& joint_name, double value);

  void setArmJointPosition(ArmSide side, int q_idx, double value);
  
  void setHandJointPosition(ArmSide side, int q_idx, double value);

  double getVariablePosition(const std::string& joint_name) const;

  void setToDefaultValues();

  void setOctomap( const octomap_msgs::Octomap& map );

  void setOctomap( const std::shared_ptr< const octomap::OcTree > &octree,
                                                                    const Eigen::Isometry3d &t );

  void setMultipleOctomaps( const std::vector< std::shared_ptr< const octomap::OcTree > > &octrees);

  void processPlanningSceneWorldMsg(const moveit_msgs::PlanningSceneWorld &world);

  void update();

  bool isStateValid() const;
  bool isStateValidAndSafe() const;

  bool isRightArmInLimits(const ArmJntArray& q) const;

  bool isLeftArmInLimits(const ArmJntArray& q) const;

  double getArmLimitDist(const ArmJntArray& q, int q_idx) const;

  double getArmLimitDistJnt(const ArmJntArray& q, int q_idx) const;

  double getWristLimitDist(const ArmJntArray& q, ArmSide side) const;

  double getTorsoLimitLo() const;

  double getTorsoLimitUp() const;

  const std::array<std::string, 7 >& getRightArmJointNames() const;

  const std::array<std::string, 7 >& getLeftArmJointNames() const;
};

#endif  // __VELMA_STATE_VALIDATOR__H__
