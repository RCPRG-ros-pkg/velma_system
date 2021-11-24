#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept>

#include <kdl/frames.hpp>
#include "velma_kinematics/velma_state_validator.hpp"
#include <collision_convex_model/collision_convex_model.h>

void VelmaStateValidator::readJointLimits(ros::NodeHandle& nh) {
    std::vector<std::string > joint_names;
    nh.getParam("/velma_core_cs/JntLimit/joint_names", joint_names);

    if (joint_names.size() == 0) {
        std::string error_msg = "ERROR: The ROS parameter \"/velma_core_cs/JntLimit/joint_names\" is empty";
        std::cout << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }

    std::vector<std::vector<double> > all_limits;
    for (int q_idx = 0; q_idx < joint_names.size(); ++q_idx) {
        std::stringstream sstr;
        sstr << "/velma_core_cs/JntLimit/limits_" << q_idx;
        std::vector<double > limits;
        nh.getParam(sstr.str(), limits);
        if (limits.size() == 0) {
            std::string error_msg = std::string("The ROS parameter \"") + sstr.str() + "\" is empty";
            std::cout << error_msg << std::endl;
            throw std::invalid_argument( error_msg );
        }
        else if (limits.size()%2 != 0) {
            std::string error_msg = std::string("Wrong number of limits in \"") + sstr.str() + "\", must be even";
            std::cout << error_msg << std::endl;
            throw std::invalid_argument( error_msg );
        }
        all_limits.push_back(limits);
    }

    std::vector<double > limit_range;
    nh.getParam("/velma_core_cs/JntLimit/limit_range", limit_range);
    if (limit_range.size() == 0) {
        std::string error_msg = "The ROS parameter \"/velma_core_cs/JntLimit/limit_range\" is empty";
        std::cout << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }

    const std::array<std::string, 2 > sides = {"left", "right"};

    for (int side_idx = 0; side_idx < sides.size(); ++side_idx) {
        const std::string& side_str = sides[side_idx];
        for (int q_idx = 0; q_idx < 7; ++q_idx) {
            std::stringstream sstr;
            sstr << side_str << "_arm_" << q_idx << "_joint";
            std::string joint_name = sstr.str();
            if (side_str == "left") {
                m_left_arm_joint_names[q_idx] = joint_name;
            }
            else {
                m_right_arm_joint_names[q_idx] = joint_name;
            }

            bool found = false;
            for (int i = 0; i < joint_names.size(); ++i) {
                if (joint_names[i] == joint_name) {
                    if (side_str == "left") {
                        m_right_arm_limits[q_idx] = all_limits[i];
                    }
                    else {
                        m_left_arm_limits[q_idx] = all_limits[i];
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                sstr << "Could not find limits for arm " << side_str << ", joint " << q_idx;
                std::string error_msg = sstr.str();
                std::cout << error_msg << std::endl;
                throw std::invalid_argument( error_msg );
            }
        }
    }

    std::cout << "limits for right arm:" << std::endl;
    for (int i = 0 ; i < m_right_arm_limits.size(); ++i) {
        std::cout << "joint " << i << ": ";
        for (int j = 0; j < m_right_arm_limits[i].size(); ++j) {
            std::cout << m_right_arm_limits[i][j] << ", ";
        }
        std::cout << std::endl;
    }

    // Get limits for torso
    bool found = false;
    for (int i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "torso_0_joint") {
            m_torso_limit_lo = all_limits[i][0];
            m_torso_limit_up = all_limits[i][1];
            found = true;
            break;
        }
    }
    if (!found) {
        std::stringstream sstr;
        sstr << "Could not find limits for torso";
        std::string error_msg = sstr.str();
        std::cout << error_msg << std::endl;
        throw std::invalid_argument( error_msg );
    }
}

bool VelmaStateValidator::isArmInLimits(const ArmJntArray& q, const ArmLimits& arm_limits) const {
    for (int q_idx = 0; q_idx < q.size(); ++q_idx) {
        bool found = false;
        for (int part_idx = 0; part_idx < arm_limits[q_idx].size(); part_idx += 2) {
            const double& lim_lo = arm_limits[q_idx][part_idx];
            const double& lim_up = arm_limits[q_idx][part_idx+1];
            if (q[q_idx] >= lim_lo && q[q_idx] <= lim_up) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

VelmaStateValidator::VelmaStateValidator(ros::NodeHandle& nh)
: m_robot_interface_loader("rcprg_planner", "rcprg_planner::RobotInterface")
{
    std::cout << "VelmaStateValidator ROS namespace: " << nh.getNamespace() << std::endl;

    std::string robot_interface_plugin_str;
    nh.getParam("robot_interface_plugin", robot_interface_plugin_str);
    if (robot_interface_plugin_str.empty()) {
        ROS_ERROR("The ROS parameter \"robot_interface_plugin\" is empty");
        throw std::invalid_argument("The ROS parameter \"robot_interface_plugin\" is empty");
    }
    ROS_INFO("Trying to load plugin: \"%s\"", robot_interface_plugin_str.c_str());

    m_robot_interface = m_robot_interface_loader.createInstance(robot_interface_plugin_str);
    ROS_INFO("Loaded plugin: \"%s\"", robot_interface_plugin_str.c_str());


    std::string robot_description_str;
    std::string robot_description_semantic_str;
    nh.getParam("/robot_description", robot_description_str);
    nh.getParam("/robot_description_semantic", robot_description_semantic_str);

    std::string xml_out;
    self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str, xml_out);

    //
    // moveit
    //
    robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_description_semantic_str) );

    m_robot_model = robot_model_loader.getModel();

    m_planning_scene.reset( new planning_scene::PlanningScene(m_robot_model) );

    m_planning_scene->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValid, m_robot_interface.get(), _1, _2) );

    // Read joint limits for cart_imp
    readJointLimits(nh);

    // Initialize state variables
    m_ss.reset(new moveit::core::RobotState(m_robot_model));
    m_ss->setToDefaultValues();
}

void VelmaStateValidator::setVariablePosition(const std::string& joint_name, double value) {
    m_ss->setVariablePosition(joint_name, value);
}

double VelmaStateValidator::getVariablePosition(const std::string& joint_name) const {
    return m_ss->getVariablePosition(joint_name);
}

void VelmaStateValidator::setToDefaultValues() {
    m_ss->setToDefaultValues();
}

void VelmaStateValidator::setOctomap( const octomap_msgs::Octomap& map ) {
    m_planning_scene->processOctomapMsg(map);
}

void VelmaStateValidator::setOctomap( const std::shared_ptr< const octomap::OcTree > &octree,
                                                                    const Eigen::Isometry3d &t ) {
    m_planning_scene->processOctomapPtr(octree, t);

    //std::cout << "VelmaStateValidator::setOctomap: printKnownObjects begin" << std::endl;
    //m_planning_scene->printKnownObjects(std::cout);
    //std::cout << "VelmaStateValidator::setOctomap: printKnownObjects end" << std::endl;
}

void VelmaStateValidator::update() {
    m_ss->update();
}

bool VelmaStateValidator::isStateValid() const {
    //return m_planning_scene->isStateValid(*m_ss, "", true);   // verbose
    return m_planning_scene->isStateValid(*m_ss);   // quiet
}

bool VelmaStateValidator::isRightArmInLimits(const ArmJntArray& q) const {
    return isArmInLimits(q, m_right_arm_limits);
}

bool VelmaStateValidator::isLeftArmInLimits(const ArmJntArray& q) const {
    return isArmInLimits(q, m_left_arm_limits);
}

double VelmaStateValidator::getTorsoLimitLo() const {
    return m_torso_limit_lo;
}

double VelmaStateValidator::getTorsoLimitUp() const {
    return m_torso_limit_up;
}


const std::array<std::string, 7 >& VelmaStateValidator::getRightArmJointNames() const {
  return m_right_arm_joint_names;
}

const std::array<std::string, 7 >& VelmaStateValidator::getLeftArmJointNames() const {
  return m_left_arm_joint_names;
}

double VelmaStateValidator::getArmLimitDist(const ArmJntArray& q, int q_idx) const {
    for (int part_idx = 0; part_idx < m_right_arm_limits[q_idx].size(); part_idx += 2) {
        const double& lim_lo = m_right_arm_limits[q_idx][part_idx];
        const double& lim_up = m_right_arm_limits[q_idx][part_idx+1];
        if (q[q_idx] >= lim_lo && q[q_idx] <= lim_up) {
            if (q[q_idx]-lim_lo < lim_up-q[q_idx]) {
                return lim_lo - q[q_idx];
            }
            else {
                return lim_up - q[q_idx];
            }
        }
    }
    return 0.0;
}

//ArmLimits VelmaStateValidator::getArmLimits() const {
//    return m_right_arm_limits;
//}
