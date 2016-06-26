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

#ifndef FT_SENSOR_GAZEBO_H__
#define FT_SENSOR_GAZEBO_H__

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

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

//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
//#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class FtSensorGazebo : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RTT::OutputPort<geometry_msgs::Wrench> port_raw_wrench_out_;
    RTT::OutputPort<geometry_msgs::Wrench> port_fast_filtered_wrench_out_;
    RTT::OutputPort<geometry_msgs::Wrench> port_slow_filtered_wrench_out_;

    geometry_msgs::Wrench raw_wrench_out_;
    geometry_msgs::Wrench fast_filtered_wrench_out_;
    geometry_msgs::Wrench slow_filtered_wrench_out_;

    // public methods
    FtSensorGazebo(std::string const& name);
    ~FtSensorGazebo();
    void updateHook();
    bool startHook();
    bool configureHook();
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);

  protected:

    void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out) const;

    ros::NodeHandle *nh_;

    // ROS parameters
    std::string joint_name_;
    std::vector<double> transform_xyz_;
    std::vector<double> transform_rpy_;

    gazebo::physics::ModelPtr model_;
//    dart::dynamics::Skeleton *dart_sk_;
    gazebo::physics::JointPtr joint_;
//    dart::dynamics::BodyNode *dart_bn_;
    gazebo::physics::LinkPtr link_;

    std::vector<KDL::Wrench> slow_buffer_;
    std::vector<KDL::Wrench> fast_buffer_;

    int slow_buffer_index_;
    int fast_buffer_index_;

    int slow_buffer_size_;
    int fast_buffer_size_;

    KDL::Wrench slow_filtered_wrench_;
    KDL::Wrench fast_filtered_wrench_;

    // properties
    std::vector<double> force_limits_;
    RTT::Property<KDL::Wrench> offset_prop_;

    KDL::Frame T_W_S_;

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;
};

#endif  // FT_SENSOR_GAZEBO_H__

