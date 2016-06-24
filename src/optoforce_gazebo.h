/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OPTOFORCE_GAZEBO_H__
#define OPTOFORCE_GAZEBO_H__

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <barrett_hand_controller_msgs/BHPressureState.h>
#include <barrett_hand_controller_msgs/BHTemp.h>
#include <barrett_hand_controller_msgs/BHPressureInfo.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/physics/dart/DARTModel.hh>
//#include <gazebo/physics/dart/DARTJoint.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/WrenchStamped.h>

#include <kuka_lwr_fri/friComm.h>
#include "barrett_hand_common/tactile.h"

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class OptoforceGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // public methods
    OptoforceGazebo(std::string const& name);
    ~OptoforceGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    ros::NodeHandle *nh_;

    // ROS parameters
    std::string device_name_;
    int n_sensors_;
    std::vector<std::string > frame_id_vec_;

    int32_t median_filter_samples_, median_filter_max_samples_;

    gazebo::physics::ModelPtr model_;
//    gazebo::physics::DARTModelPtr model_dart_;
//    dart::dynamics::Skeleton *dart_sk_;
//    dart::simulation::World *dart_world_;
//    dart::collision::CollisionDetector* detector_;

    gazebo::physics::JointControllerPtr jc_;
    std::vector<gazebo::physics::JointPtr> joints_;

    // OROCOS ports
    std::vector< RTT::OutputPort<geometry_msgs::WrenchStamped >* > port_force_out_;

    // port variables
    std::vector< geometry_msgs::WrenchStamped > force_out_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;
};

#endif  // OPTOFORCE_GAZEBO_H__

