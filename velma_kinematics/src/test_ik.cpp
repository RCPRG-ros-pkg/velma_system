#include <iostream>
#include <algorithm>
#include <sstream>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <kdl/frames.hpp>

#include "velma_kinematics/velma_kinematics.hpp"
#include "velma_kinematics/velma_state_validator.hpp"

#include <sensor_msgs/JointState.h>

#include <rcprg_ros_utils/marker_publisher.h>

double deg2rad(double angle_deg) {
  return angle_deg/180.0*3.1415;
}

double randUniform(double min, double max) {
  return min + (max - min) * double(rand())/double(RAND_MAX);
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "test_ik");
    ros::NodeHandle m_nh;

    MarkerPublisher markerPub(m_nh, "/velma_ik_geom");
    ros::Publisher jsPub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 10, true);
    ros::Duration(0.5).sleep();

    const std::string arm_side = "left";

    std::vector<std::string > arm_joint_names_suffixes = {"_arm_0_joint", "_arm_1_joint",
        "_arm_2_joint", "_arm_3_joint", "_arm_4_joint", "_arm_5_joint", "_arm_6_joint"};

    std::vector<std::string > arm_joint_names;
    for (int q_idx = 0; q_idx < arm_joint_names_suffixes.size(); ++q_idx) {
      arm_joint_names.push_back( arm_side + arm_joint_names_suffixes[q_idx] );
    }

    sensor_msgs::JointState js;
    js.name.resize( arm_joint_names.size() + 1 );
    js.position.resize( arm_joint_names.size() + 1 );

    js.name[0] = "torso_0_joint";
    for (int q_idx = 0; q_idx < arm_joint_names.size(); ++q_idx) {
      const std::string& joint_name = arm_joint_names[q_idx];
        js.name[q_idx+1] = joint_name;
    }

    KDL::Frame T_B_Ws;
    KDL::Frame T_B_We(KDL::Rotation(), KDL::Vector(0.6, 0, 1.3));
    KDL::Twist twist;

    VelmaStateValidatorLoader vsv_loader(m_nh);
    VelmaStateValidatorPtr vsv = vsv_loader.create();

    double torso_angle = 0.5;

    KinematicsSolverVelmaLoader v_solv_loader(m_nh);
    KinematicsSolverVelmaPtr v_solv = v_solv_loader.create();

    double traj_factor = 2.0;
    while (ros::ok()) {
      if (traj_factor >= 1.0) {
        T_B_Ws = T_B_We;
        T_B_We = KDL::Frame( KDL::Rotation::RPY( randUniform(-3.14, 3.14), randUniform(-3.14, 3.14), randUniform(-3.14, 3.14) ),
            KDL::Vector(randUniform(0.2, 0.7), randUniform(-0.5, 0.5), randUniform(0.8, 1.8)) );
        traj_factor = 0.0;
        twist = KDL::diff(T_B_Ws, T_B_We, 1);
      }
      KDL::Frame T_B_Wd = KDL::addDelta(T_B_Ws, twist, traj_factor);
      traj_factor += 0.01;

      KinematicsSolverLWR4::Solutions ik_solutions(v_solv->getMaximumSolutionsCount());
      int ik_solutions_count;

      bool ik_possible = false;
      if (arm_side == "left") {
        ik_possible = v_solv->calculateIkSetArm(KinematicsSolverVelma::LEFT, torso_angle, T_B_Wd,
                                                                ik_solutions, ik_solutions_count);
      }
      else {
        ik_possible = v_solv->calculateIkSetArm(KinematicsSolverVelma::RIGHT, torso_angle, T_B_Wd,
                                                                ik_solutions, ik_solutions_count);
      }

      if (ik_possible) {
        int valid_solutions = 0;
        // Visualize all solutions
        for (int i = 0; i < ik_solutions_count; ++i) {
          if (!ros::ok()) {
            break;
          }
          bool is_in_limits = false;
          if (arm_side == "left") {
            is_in_limits = vsv->isLeftArmInLimits(ik_solutions[i].q);
          }
          else {
            is_in_limits = vsv->isRightArmInLimits(ik_solutions[i].q);
          }
          if (is_in_limits) {
            for (int q_idx = 0; q_idx < arm_joint_names.size(); ++q_idx) {
              vsv->setVariablePosition(arm_joint_names[q_idx], ik_solutions[i].q[q_idx]);
            }
            vsv->update();

            if (!vsv->isStateValid()) {
              continue;
            }

            ++valid_solutions;
            js.position[0] = torso_angle;
            for (int q_idx = 0; q_idx < ik_solutions[i].q.size(); ++q_idx) {
              js.position[q_idx+1] = ik_solutions[i].q[q_idx];
            }
            js.header.stamp = ros::Time::now();
            jsPub.publish(js);

            ros::Duration(0.01).sleep();
            ros::spinOnce();
          }
        }
        std::cout << "valid_solutions: " << valid_solutions << std::endl;
      }

      markerPub.addFrameMarker(T_B_Wd, 0, 0.2, "torso_base");
      markerPub.publish();
      ros::Duration(0.04).sleep();
      ros::spinOnce();
    }

    return 0;
}
