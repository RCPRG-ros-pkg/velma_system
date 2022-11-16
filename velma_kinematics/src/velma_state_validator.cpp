#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept>

#include <kdl/frames.hpp>
#include "velma_kinematics/velma_state_validator.hpp"
#include <collision_convex_model/collision_convex_model.h>

const std::array<std::string, 7 > VelmaStateValidator::m_right_arm_joint_names({"right_arm_0_joint",
                                "right_arm_1_joint", "right_arm_2_joint", "right_arm_3_joint",
                                "right_arm_4_joint", "right_arm_5_joint", "right_arm_6_joint"});

const std::array<std::string, 7 > VelmaStateValidator::m_left_arm_joint_names({"left_arm_0_joint",
                                "left_arm_1_joint", "left_arm_2_joint", "left_arm_3_joint",
                                "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"});

const std::array<std::string, 8 > VelmaStateValidator::m_right_hand_joint_names({
        "right_HandFingerOneKnuckleTwoJoint",
        "right_HandFingerTwoKnuckleTwoJoint", "right_HandFingerThreeKnuckleTwoJoint",
        "right_HandFingerOneKnuckleOneJoint", "right_HandFingerOneKnuckleThreeJoint",
        "right_HandFingerTwoKnuckleThreeJoint", "right_HandFingerThreeKnuckleThreeJoint",
        "right_HandFingerTwoKnuckleOneJoint"});
  
const std::array<std::string, 8 > VelmaStateValidator::m_left_hand_joint_names({
        "left_HandFingerOneKnuckleTwoJoint",
        "left_HandFingerTwoKnuckleTwoJoint", "left_HandFingerThreeKnuckleTwoJoint",
        "left_HandFingerOneKnuckleOneJoint", "left_HandFingerOneKnuckleThreeJoint",
        "left_HandFingerTwoKnuckleThreeJoint", "left_HandFingerThreeKnuckleThreeJoint",
        "left_HandFingerTwoKnuckleOneJoint"});


VelmaStateValidatorLoader::VelmaStateValidatorLoader(ros::NodeHandle& nh)
: m_robot_interface_loader("rcprg_planner", "rcprg_planner::RobotInterface")
, m_wcc_l_joint0_idx(-1)
, m_wcc_l_joint1_idx(-1)
, m_wcc_r_joint0_idx(-1)
, m_wcc_r_joint1_idx(-1)
, m_wcc_r_d0(0)
, m_wcc_l_d0(0)
{
    std::cout << "VelmaStateValidator ROS namespace: " << nh.getNamespace() << std::endl;

    nh.getParam("robot_interface_plugin", m_robot_interface_plugin_str);
    if (m_robot_interface_plugin_str.empty()) {
        ROS_ERROR("The ROS parameter \"robot_interface_plugin\" is empty");
        throw std::invalid_argument("The ROS parameter \"robot_interface_plugin\" is empty");
    }
    ROS_INFO("VelmaStateValidator: Trying to load plugin: \"%s\"", m_robot_interface_plugin_str.c_str());

    std::string tmp_robot_description_str;
    nh.getParam("/robot_description", tmp_robot_description_str);
    nh.getParam("/robot_description_semantic", m_robot_description_semantic_str);

    self_collision::CollisionModel::convertSelfCollisionsInURDF(tmp_robot_description_str,
                                                                        m_robot_description_str);

    // Read joint limits for cart_imp
    readJointLimits(nh);

    nh.getParam("velma_core_cs/wcc_l/constraint_polygon", m_wcc_l_constraint_polygon);
    nh.getParam("velma_core_cs/wcc_l/joint0_idx", m_wcc_l_joint0_idx);
    nh.getParam("velma_core_cs/wcc_l/joint1_idx", m_wcc_l_joint1_idx);

    nh.getParam("velma_core_cs/wcc_r/constraint_polygon", m_wcc_r_constraint_polygon);
    nh.getParam("velma_core_cs/wcc_r/joint0_idx", m_wcc_r_joint0_idx);
    nh.getParam("velma_core_cs/wcc_r/joint1_idx", m_wcc_r_joint1_idx);

    nh.getParam("velma_core_cs/wcc_r/d0", m_wcc_r_d0);
    nh.getParam("velma_core_cs/wcc_l/d0", m_wcc_l_d0);

    if (m_wcc_l_constraint_polygon.size() == 0 || (m_wcc_l_constraint_polygon.size()%2) != 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_l/constraint_polygon\' (l) has wrong size: %lu", m_wcc_l_constraint_polygon.size());
        throw std::runtime_error("error");
    }

    if (m_wcc_l_d0 == 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_l/d0\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_r_d0 == 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_r/d0\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_l_joint0_idx < 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_l/joint0_idx\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_l_joint1_idx < 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_l/joint1_idx\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_r_joint0_idx < 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_r/joint0_idx\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_r_joint1_idx < 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_r/joint1_idx\' is not set");
        throw std::runtime_error("error");
    }

    if (m_wcc_l_joint0_idx == m_wcc_l_joint1_idx) {
        ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (l) have the same value: %d", m_wcc_l_joint0_idx);
        throw std::runtime_error("error");
    }

    if (m_wcc_r_constraint_polygon.size() == 0 || (m_wcc_r_constraint_polygon.size()%2) != 0) {
        ROS_ERROR("property \'/velma_core_cs/wcc_r/constraint_polygon\' (r) has wrong size: %lu", m_wcc_r_constraint_polygon.size());
        throw std::runtime_error("error");
    }

    if (m_wcc_r_joint0_idx == m_wcc_r_joint1_idx) {
        ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (r) have the same value: %d", m_wcc_r_joint0_idx);
        throw std::runtime_error("error");
    }
}

void VelmaStateValidatorLoader::readJointLimits(ros::NodeHandle& nh) {
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
        const std::array<std::string, 7 >& arm_joint_names = ((side_str == "left")?
                                                VelmaStateValidator::m_left_arm_joint_names :
                                                VelmaStateValidator::m_right_arm_joint_names);

        for (int q_idx = 0; q_idx < 7; ++q_idx) {
            const std::string& joint_name = arm_joint_names[q_idx];

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
                std::stringstream sstr;
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

VelmaStateValidatorPtr VelmaStateValidatorLoader::create() {
    boost::shared_ptr<rcprg_planner::RobotInterface> robot_interface =
                        m_robot_interface_loader.createInstance(m_robot_interface_plugin_str);
    ROS_INFO("VelmaStateValidatorLoader: Loaded plugin: \"%s\"", m_robot_interface_plugin_str.c_str());

    //
    // moveit
    //
    robot_model_loader::RobotModelLoader robot_model_loader(
                        robot_model_loader::RobotModelLoader::Options(m_robot_description_str,
                                                            m_robot_description_semantic_str) );

    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // bool dbg_print_srdf_groups = false;
    // if (dbg_print_srdf_groups) {
    //     std::cout << "srdf groups:" << std::endl;
    //     const std::vector< std::string >& group_names = robot_model->getJointModelGroupNames();

    //     for (int i = 0; i < group_names.size(); ++i) {
    //         std::cout << "  " << group_names[i] << std::endl;
    //         robot_model::JointModelGroup *joint_group = robot_model->getJointModelGroup(group_names[i]);
    //         const std::vector< const robot_model::LinkModel * > &link_models = joint_group->getLinkModels();
    //         for (int j = 0; j < link_models.size(); ++j) {
    //             std::cout << "    " << link_models[j]->getName() << std::endl;
    //         }
    //     }
    // }

    // // TODO: add interface methods to add objects to the scene

    // m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(robot_model) ) );

    // m_planning_scenes[0]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValid, m_robot_interface.get(), _1, _2) );

    // std::cout << "VelmaStateValidator: Using collision detector: \""
    //         << m_planning_scenes[0]->getActiveCollisionDetectorName()
    //         << "\"" << std::endl;

    // // Read joint limits for cart_imp
    // readJointLimits(nh);

    // // Initialize state variables
    // m_ss.reset(new moveit::core::RobotState(robot_model));
    // m_ss->setToDefaultValues();

    std::shared_ptr<DoubleJointCC > wcc_l(new DoubleJointCC(m_wcc_l_d0, m_wcc_l_constraint_polygon));
    std::shared_ptr<DoubleJointCC > wcc_r(new DoubleJointCC(m_wcc_r_d0, m_wcc_r_constraint_polygon));

    VelmaStateValidatorPtr result( new VelmaStateValidator(robot_interface,
                                                            robot_model,
                                                            wcc_l,
                                                            wcc_r));

    result->m_right_arm_limits = m_right_arm_limits;;
    result->m_left_arm_limits = m_left_arm_limits;

    result->m_torso_limit_lo = m_torso_limit_lo;
    result->m_torso_limit_up = m_torso_limit_up;

    return result;
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

VelmaStateValidator::VelmaStateValidator(
                            boost::shared_ptr<rcprg_planner::RobotInterface> robot_interface,
                            robot_model::RobotModelPtr robot_model,
                            std::shared_ptr<DoubleJointCC > wcc_l,
                            std::shared_ptr<DoubleJointCC > wcc_r)
    : m_verbose(false)
    , m_robot_interface(robot_interface)
    , m_robot_model(robot_model)
    , m_wcc_l(wcc_l)
    , m_wcc_r(wcc_r)
{
    bool dbg_print_srdf_groups = false;
    if (dbg_print_srdf_groups) {
        std::cout << "srdf groups:" << std::endl;
        const std::vector< std::string >& group_names = m_robot_model->getJointModelGroupNames();

        for (int i = 0; i < group_names.size(); ++i) {
            std::cout << "  " << group_names[i] << std::endl;
            robot_model::JointModelGroup *joint_group = m_robot_model->getJointModelGroup(group_names[i]);
            const std::vector< const robot_model::LinkModel * > &link_models = joint_group->getLinkModels();
            for (int j = 0; j < link_models.size(); ++j) {
                std::cout << "    " << link_models[j]->getName() << std::endl;
            }
        }
    }

    // TODO: add interface methods to add objects to the scene

    m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(m_robot_model) ) );

    m_planning_scenes[0]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValid, m_robot_interface.get(), _1, _2) );

    std::cout << "VelmaStateValidator: Using collision detector: \""
            << m_planning_scenes[0]->getActiveCollisionDetectorName()
            << "\"" << std::endl;

    // Read joint limits for cart_imp
    //readJointLimits(nh);

    // Initialize state variables
    m_ss.reset(new moveit::core::RobotState(m_robot_model));
    m_ss->setToDefaultValues();
}


// VelmaStateValidator::VelmaStateValidator(ros::NodeHandle& nh)
    // //: m_robot_interface_loader("rcprg_planner", "rcprg_planner::RobotInterface")
    // : m_verbose(false)
    // , wcc_l_joint0_idx_(-1)
    // , wcc_l_joint1_idx_(-1)
    // , wcc_r_joint0_idx_(-1)
    // , wcc_r_joint1_idx_(-1)
    // , wcc_r_d0_(0)
    // , wcc_l_d0_(0)
    // , m_right_arm_joint_names({"right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint",
    //     "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint", "right_arm_6_joint"})
    // , m_left_arm_joint_names({"left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint",
    //     "left_arm_3_joint", "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"})
    // , m_right_hand_joint_names({"right_HandFingerOneKnuckleTwoJoint",
    //     "right_HandFingerTwoKnuckleTwoJoint", "right_HandFingerThreeKnuckleTwoJoint",
    //     "right_HandFingerOneKnuckleOneJoint", "right_HandFingerOneKnuckleThreeJoint",
    //     "right_HandFingerTwoKnuckleThreeJoint", "right_HandFingerThreeKnuckleThreeJoint",
    //     "right_HandFingerTwoKnuckleOneJoint"})
    // , m_left_hand_joint_names({"left_HandFingerOneKnuckleTwoJoint",
    //     "left_HandFingerTwoKnuckleTwoJoint", "left_HandFingerThreeKnuckleTwoJoint",
    //     "left_HandFingerOneKnuckleOneJoint", "left_HandFingerOneKnuckleThreeJoint",
    //     "left_HandFingerTwoKnuckleThreeJoint", "left_HandFingerThreeKnuckleThreeJoint",
    //     "left_HandFingerTwoKnuckleOneJoint"})
// {
    // pluginlib::ClassLoader<rcprg_planner::RobotInterface>
    //                     robot_interface_loader("rcprg_planner", "rcprg_planner::RobotInterface");

    // std::cout << "VelmaStateValidator ROS namespace: " << nh.getNamespace() << std::endl;

    // std::string robot_interface_plugin_str;
    // nh.getParam("robot_interface_plugin", robot_interface_plugin_str);
    // if (robot_interface_plugin_str.empty()) {
    //     ROS_ERROR("The ROS parameter \"robot_interface_plugin\" is empty");
    //     throw std::invalid_argument("The ROS parameter \"robot_interface_plugin\" is empty");
    // }
    // ROS_INFO("VelmaStateValidator: Trying to load plugin: \"%s\"", robot_interface_plugin_str.c_str());

    // m_robot_interface = m_robot_interface_loader.createInstance(robot_interface_plugin_str);
    // ROS_INFO("VelmaStateValidator: Loaded plugin: \"%s\"", robot_interface_plugin_str.c_str());


    // std::string robot_description_str;
    // std::string robot_description_semantic_str;
    // nh.getParam("/robot_description", robot_description_str);
    // nh.getParam("/robot_description_semantic", robot_description_semantic_str);

    // std::string xml_out;
    // self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str, xml_out);

    // //
    // // moveit
    // //
    // robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_description_semantic_str) );

    // m_robot_model = robot_model_loader.getModel();

//     bool dbg_print_srdf_groups = false;
//     if (dbg_print_srdf_groups) {
//         std::cout << "srdf groups:" << std::endl;
//         const std::vector< std::string >& group_names = m_robot_model->getJointModelGroupNames();

//         for (int i = 0; i < group_names.size(); ++i) {
//             std::cout << "  " << group_names[i] << std::endl;
//             robot_model::JointModelGroup *joint_group = m_robot_model->getJointModelGroup(group_names[i]);
//             const std::vector< const robot_model::LinkModel * > &link_models = joint_group->getLinkModels();
//             for (int j = 0; j < link_models.size(); ++j) {
//                 std::cout << "    " << link_models[j]->getName() << std::endl;
//             }
//         }
//     }

//     // TODO: add interface methods to add objects to the scene

//     m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(m_robot_model) ) );

//     m_planning_scenes[0]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValid, m_robot_interface.get(), _1, _2) );

//     std::cout << "VelmaStateValidator: Using collision detector: \""
//             << m_planning_scenes[0]->getActiveCollisionDetectorName()
//             << "\"" << std::endl;

//     // Read joint limits for cart_imp
//     readJointLimits(nh);

//     // Initialize state variables
//     m_ss.reset(new moveit::core::RobotState(m_robot_model));
//     m_ss->setToDefaultValues();

//     nh.getParam("velma_core_cs/wcc_l/constraint_polygon", wcc_l_constraint_polygon_);
//     nh.getParam("velma_core_cs/wcc_l/joint0_idx", wcc_l_joint0_idx_);
//     nh.getParam("velma_core_cs/wcc_l/joint1_idx", wcc_l_joint1_idx_);

//     nh.getParam("velma_core_cs/wcc_r/constraint_polygon", wcc_r_constraint_polygon_);
//     nh.getParam("velma_core_cs/wcc_r/joint0_idx", wcc_r_joint0_idx_);
//     nh.getParam("velma_core_cs/wcc_r/joint1_idx", wcc_r_joint1_idx_);

//     nh.getParam("velma_core_cs/wcc_r/d0", wcc_r_d0_);
//     nh.getParam("velma_core_cs/wcc_l/d0", wcc_l_d0_);

//     if (wcc_l_constraint_polygon_.size() == 0 || (wcc_l_constraint_polygon_.size()%2) != 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_l/constraint_polygon\' (l) has wrong size: %lu", wcc_l_constraint_polygon_.size());
//         throw std::runtime_error("error");
//     }

//     if (wcc_l_d0_ == 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_l/d0\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_r_d0_ == 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_r/d0\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_l_joint0_idx_ < 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_l/joint0_idx\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_l_joint1_idx_ < 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_l/joint1_idx\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_r_joint0_idx_ < 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_r/joint0_idx\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_r_joint1_idx_ < 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_r/joint1_idx\' is not set");
//         throw std::runtime_error("error");
//     }

//     if (wcc_l_joint0_idx_ == wcc_l_joint1_idx_) {
//         ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (l) have the same value: %d", wcc_l_joint0_idx_);
//         throw std::runtime_error("error");
//     }

//     if (wcc_r_constraint_polygon_.size() == 0 || (wcc_r_constraint_polygon_.size()%2) != 0) {
//         ROS_ERROR("property \'/velma_core_cs/wcc_r/constraint_polygon\' (r) has wrong size: %lu", wcc_r_constraint_polygon_.size());
//         throw std::runtime_error("error");
//     }

//     if (wcc_r_joint0_idx_ == wcc_r_joint1_idx_) {
//         ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (r) have the same value: %d", wcc_r_joint0_idx_);
//         throw std::runtime_error("error");
//     }

//     wcc_l_.reset(new DoubleJointCC(wcc_l_d0_, wcc_l_constraint_polygon_));
//     wcc_r_.reset(new DoubleJointCC(wcc_r_d0_, wcc_r_constraint_polygon_));
// }

void VelmaStateValidator::setVariablePosition(const std::string& joint_name, double value) {
    m_ss->setVariablePosition(joint_name, value);
}

void VelmaStateValidator::setArmJointPosition(ArmSide side, int q_idx, double value) {
    if (side == ARM_R) {
        m_ss->setVariablePosition(m_right_arm_joint_names[q_idx], value);
    }
    else if (side == ARM_L) {
        m_ss->setVariablePosition(m_left_arm_joint_names[q_idx], value);
    }
}

void VelmaStateValidator::setHandJointPosition(ArmSide side, int q_idx, double value) {
    const std::array<std::string, 8 >& joint_names =
                                ((side == ARM_R)?m_right_hand_joint_names:m_left_hand_joint_names);
    if (q_idx == 0) {
        // f1
        m_ss->setVariablePosition(joint_names[0], value);
        m_ss->setVariablePosition(joint_names[4], value*0.333333);
    }
    else if (q_idx == 1) {
        // f2
        m_ss->setVariablePosition(joint_names[1], value);
        m_ss->setVariablePosition(joint_names[5], value*0.333333);
    }
    else if (q_idx == 2) {
        // f3
        m_ss->setVariablePosition(joint_names[2], value);
        m_ss->setVariablePosition(joint_names[6], value*0.333333);
    }
    else if (q_idx == 3) {
        // sp
        m_ss->setVariablePosition(joint_names[3], value);
        m_ss->setVariablePosition(joint_names[7], value*0.333333);
    }
    else {
        std::stringstream sstr;
        sstr << "ERROR: wrong index for hand joint: " << q_idx;
        std::string error_msg = sstr.str();
        std::cout << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }
}

double VelmaStateValidator::getVariablePosition(const std::string& joint_name) const {
    return m_ss->getVariablePosition(joint_name);
}

void VelmaStateValidator::setToDefaultValues() {
    m_ss->setToDefaultValues();
}

void VelmaStateValidator::setOctomap( const octomap_msgs::Octomap& map ) {
    if (m_planning_scenes.size() != 1) {
        m_planning_scenes.clear();
        m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(m_robot_model) ) );
        m_planning_scenes[0]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValidAndSafe, m_robot_interface.get(), _1, _2) );
    }
    m_planning_scenes[0]->processOctomapMsg(map);
}

void VelmaStateValidator::setOctomap( const std::shared_ptr< const octomap::OcTree > &octree,
                                                                    const Eigen::Isometry3d &t ) {
    if (m_planning_scenes.size() != 1) {
        m_planning_scenes.clear();
        m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(m_robot_model) ) );
        m_planning_scenes[0]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValidAndSafe, m_robot_interface.get(), _1, _2) );
    }
    m_planning_scenes[0]->processOctomapPtr(octree, t);

    //std::cout << "VelmaStateValidator::setOctomap: printKnownObjects begin" << std::endl;
    //m_planning_scene->printKnownObjects(std::cout);
    //std::cout << "VelmaStateValidator::setOctomap: printKnownObjects end" << std::endl;
}

void VelmaStateValidator::setMultipleOctomaps( const std::vector< std::shared_ptr< const octomap::OcTree > > &octrees) {
    m_planning_scenes.clear();
    for (int i = 0; i < octrees.size(); ++i) {
        m_planning_scenes.push_back( planning_scene::PlanningScenePtr( new planning_scene::PlanningScene(m_robot_model) ) );
        m_planning_scenes[i]->setStateFeasibilityPredicate( boost::bind(&rcprg_planner::RobotInterface::isStateValidAndSafe, m_robot_interface.get(), _1, _2) );
        m_planning_scenes[i]->processOctomapPtr(octrees[i], Eigen::Isometry3d::Identity());
    }
}

void VelmaStateValidator::update() {
    m_ss->update();
}

bool VelmaStateValidator::isStateValid() const {
    for (int i = 0; i < m_planning_scenes.size(); ++i) {
        if (m_planning_scenes[i]->isStateValid(*m_ss, "", m_verbose)) {
            return true;
        }
    }
    return false;
}

bool VelmaStateValidator::isStateValidAndSafe() const {
    // TODO: check soft joint limits
    const ArmJntArray q_r({
            m_ss->getVariablePosition("right_arm_0_joint"),
            m_ss->getVariablePosition("right_arm_1_joint"),
            m_ss->getVariablePosition("right_arm_2_joint"),
            m_ss->getVariablePosition("right_arm_3_joint"),
            m_ss->getVariablePosition("right_arm_4_joint"),
            m_ss->getVariablePosition("right_arm_5_joint"),
            m_ss->getVariablePosition("right_arm_6_joint")});
    const ArmJntArray q_l({
            m_ss->getVariablePosition("left_arm_0_joint"),
            m_ss->getVariablePosition("left_arm_1_joint"),
            m_ss->getVariablePosition("left_arm_2_joint"),
            m_ss->getVariablePosition("left_arm_3_joint"),
            m_ss->getVariablePosition("left_arm_4_joint"),
            m_ss->getVariablePosition("left_arm_5_joint"),
            m_ss->getVariablePosition("left_arm_6_joint")});
    const double ang_lim = 10.0/180.0*M_PI;
    for (int q_idx = 0; q_idx < 7; ++q_idx) {
        if (getArmLimitDistJnt(q_r, q_idx) < ang_lim) {
            return false;
        }
        if (getArmLimitDistJnt(q_l, q_idx) < ang_lim) {
            return false;
        }
    }
    return isStateValid();
}

void VelmaStateValidator::setVerbose(bool verbose) {
    m_verbose = verbose;
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

double VelmaStateValidator::getArmLimitDistJnt(const ArmJntArray& q, int q_idx) const {
    const double& lim_lo = m_right_arm_limits[q_idx][0];
    const double& lim_up = m_right_arm_limits[q_idx][m_right_arm_limits[q_idx].size()-1];
    if (q[q_idx] >= lim_lo && q[q_idx] <= lim_up) {
        if (q[q_idx]-lim_lo < lim_up-q[q_idx]) {
            return lim_lo - q[q_idx];
        }
        else {
            return lim_up - q[q_idx];
        }
    }
    return 0.0;
}

double VelmaStateValidator::getWristLimitDist(const ArmJntArray& q, ArmSide side) const {
    DoubleJointCC::Joints min_v;
    double min_dist = 0.0;
    int min_idx;
    int min_type;
    if (side == ARM_R) {
        DoubleJointCC::Joints q_wrist( q[5], q[6] );
        if ( m_wcc_r->getMinDistanceIn(q_wrist, min_v, min_dist, min_idx, min_type) ) {
            return min_dist;
        }
    }
    else if (side == ARM_L) {
        DoubleJointCC::Joints q_wrist( q[5], q[6] );
        if ( m_wcc_l->getMinDistanceIn(q_wrist, min_v, min_dist, min_idx, min_type) ) {
            return min_dist;
        }
    }
    return 1.0;
}

//ArmLimits VelmaStateValidator::getArmLimits() const {
//    return m_right_arm_limits;
//}
