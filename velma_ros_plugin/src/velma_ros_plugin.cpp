// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//


#include <pluginlib/class_list_macros.h>
#include <rcprg_planner/robot_interface.h>
#include <planer_utils/double_joint_collision_checker.h>

namespace velma_planner {
    class VelmaInterface : public rcprg_planner::RobotInterface {
    protected:
        ros::NodeHandle nh_;

        // ROS parameters
        std::vector<double > wcc_l_constraint_polygon_;
        int wcc_l_joint0_idx_;
        int wcc_l_joint1_idx_;

        std::vector<double > wcc_r_constraint_polygon_;
        int wcc_r_joint0_idx_;
        int wcc_r_joint1_idx_;

        boost::shared_ptr<DoubleJointCC > wcc_l_;
        boost::shared_ptr<DoubleJointCC > wcc_r_;

    public:
        VelmaInterface()
            : nh_()
        {
            nh_.getParam("/velma_core_cs/wcc_l/constraint_polygon", wcc_l_constraint_polygon_);
            nh_.getParam("/velma_core_cs/wcc_l/joint0_idx", wcc_l_joint0_idx_);
            nh_.getParam("/velma_core_cs/wcc_l/joint1_idx", wcc_l_joint1_idx_);

            nh_.getParam("/velma_core_cs/wcc_r/constraint_polygon", wcc_r_constraint_polygon_);
            nh_.getParam("/velma_core_cs/wcc_r/joint0_idx", wcc_r_joint0_idx_);
            nh_.getParam("/velma_core_cs/wcc_r/joint1_idx", wcc_r_joint1_idx_);

            if (wcc_l_constraint_polygon_.size() == 0 || (wcc_l_constraint_polygon_.size()%2) != 0) {
                ROS_ERROR("property \'constraint_polygon\' (l) has wrong size: %lu", wcc_l_constraint_polygon_.size());
            }

            if (wcc_l_joint0_idx_ == wcc_l_joint1_idx_) {
                ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (l) have the same value: %d", wcc_l_joint0_idx_);
            }

            if (wcc_r_constraint_polygon_.size() == 0 || (wcc_r_constraint_polygon_.size()%2) != 0) {
                ROS_ERROR("property \'constraint_polygon\' (r) has wrong size: %lu", wcc_r_constraint_polygon_.size());
            }

            if (wcc_r_joint0_idx_ == wcc_r_joint1_idx_) {
                ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (r) have the same value: %d", wcc_r_joint0_idx_);
            }

            wcc_l_.reset(new DoubleJointCC(0.0, wcc_l_constraint_polygon_));
            wcc_r_.reset(new DoubleJointCC(0.0, wcc_r_constraint_polygon_));

        }

        virtual bool isStateValid(const robot_state::RobotState& ss, bool verbose) {
            DoubleJointCC::Joints q_r(ss.getVariablePosition("right_arm_5_joint"), ss.getVariablePosition("right_arm_6_joint"));
            if ( wcc_r_->inCollision(q_r) ) {
                return false;
            }

            DoubleJointCC::Joints q_l(ss.getVariablePosition("left_arm_5_joint"), ss.getVariablePosition("left_arm_6_joint"));
            if ( wcc_l_->inCollision(q_l) ) {
                return false;
            }

            return true;
        }
    };
}

PLUGINLIB_EXPORT_CLASS(velma_planner::VelmaInterface, rcprg_planner::RobotInterface)

