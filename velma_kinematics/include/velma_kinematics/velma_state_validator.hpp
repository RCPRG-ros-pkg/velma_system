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

class VelmaStateValidator {
public:
  enum ArmSide {ARM_R=0, ARM_L=1};
  typedef std::array<double, 7> ArmJntArray;

protected:
  pluginlib::ClassLoader<rcprg_planner::RobotInterface> m_robot_interface_loader;
  planning_scene::PlanningScenePtr m_planning_scene;
  robot_model::RobotModelPtr m_robot_model;
  std::shared_ptr<moveit::core::RobotState > m_ss;
  boost::shared_ptr<rcprg_planner::RobotInterface> m_robot_interface;

  typedef std::array<std::vector<double>, 7 > ArmLimits;
  ArmLimits m_right_arm_limits;
  ArmLimits m_left_arm_limits;

  double m_torso_limit_lo;
  double m_torso_limit_up;

  std::array<std::string, 7 > m_right_arm_joint_names;
  std::array<std::string, 7 > m_left_arm_joint_names;

  void readJointLimits(ros::NodeHandle& nh);

  bool isArmInLimits(const ArmJntArray& q, const ArmLimits& arm_limits) const;

public:

  VelmaStateValidator(ros::NodeHandle& nh);

  void setVariablePosition(const std::string& joint_name, double value);

  double getVariablePosition(const std::string& joint_name) const;

  void setToDefaultValues();

  void setOctomap( const octomap_msgs::Octomap& map );

  void setOctomap( const std::shared_ptr< const octomap::OcTree > &octree,
                                                                    const Eigen::Isometry3d &t );

  void update();

  bool isStateValid() const;

  bool isRightArmInLimits(const ArmJntArray& q) const;

  bool isLeftArmInLimits(const ArmJntArray& q) const;

  double getArmLimitDist(const ArmJntArray& q, int q_idx) const;

  double getTorsoLimitLo() const;

  double getTorsoLimitUp() const;

  const std::array<std::string, 7 >& getRightArmJointNames() const;

  const std::array<std::string, 7 >& getLeftArmJointNames() const;
};

#endif  // __VELMA_STATE_VALIDATOR__H__
