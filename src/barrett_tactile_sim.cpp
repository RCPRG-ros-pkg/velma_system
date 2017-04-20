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

#include "barrett_hand_common/tactile_geometry.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <barrett_hand_controller_msgs/BHPressureState.h>
#include <barrett_hand_controller_msgs/BHTemp.h>
#include <barrett_hand_controller_msgs/BHPressureInfo.h>

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include "barrett_hand_common/tactile.h"

class BarrettTactileSim : public RTT::TaskContext
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // public methods
    BarrettTactileSim(std::string const& name);
    ~BarrettTactileSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    std::string prefix_;

    int32_t median_filter_samples_, median_filter_max_samples_;

    bool data_valid_;

    std::vector<std::string > link_names_;
    std::vector<Eigen::Isometry3d > vec_T_C_L_;

    Tactile *ts_[4];

    // OROCOS ports
    RTT::OutputPort<barrett_hand_controller_msgs::BHPressureState> port_tactile_out_;
    RTT::InputPort<std_msgs::Empty> port_reset_in_;
    RTT::InputPort<std_msgs::Empty> port_calibrate_in_;
    RTT::InputPort<std_msgs::Int32> port_filter_in_;
    RTT::OutputPort<barrett_hand_controller_msgs::BHPressureInfo> port_tactile_info_out_;
    RTT::OutputPort<Eigen::Vector4d > port_max_pressure_out_;

    // port variables
    barrett_hand_controller_msgs::BHPressureState tactile_out_;
    std_msgs::Empty reset_in_;
    std_msgs::Empty calibrate_in_;
    std_msgs::Int32 filter_in_;
    barrett_hand_controller_msgs::BHPressureInfo pressure_info_;
    Eigen::Vector4d max_pressure_out_;
};

using namespace RTT;

    BarrettTactileSim::BarrettTactileSim(std::string const& name)
        : TaskContext(name, RTT::TaskContext::PreOperational)
        , median_filter_samples_(1)
        , median_filter_max_samples_(8)
        , data_valid_(false)
        , port_tactile_out_("BHPressureState_OUTPORT", false)
        , port_tactile_info_out_("tactile_info_OUTPORT", false)
        , port_max_pressure_out_("max_measured_pressure_OUTPORT", false)
    {
        ts_[0] = new Tactile(median_filter_max_samples_);
        ts_[1] = new Tactile(median_filter_max_samples_);
        ts_[2] = new Tactile(median_filter_max_samples_);
        ts_[3] = new Tactile(median_filter_max_samples_);
        ts_[0]->setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[1]->setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[2]->setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[3]->setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

        this->ports()->addPort(port_tactile_out_);
        this->ports()->addPort("calibrate_tactile_sensors_INPORT", port_calibrate_in_);
        this->ports()->addPort("set_median_filter_INPORT", port_filter_in_);
        this->ports()->addPort(port_tactile_info_out_);
        this->ports()->addPort(port_max_pressure_out_);
        max_pressure_out_.setZero();

		this->addProperty("prefix", prefix_);

        // tactile array info
        for (int id=0; id<4; ++id) {
            pressure_info_.sensor[id].frame_id = ts_[id]->getName();
            for (int i=0; i<24; ++i)
            {
                pressure_info_.sensor[id].force_per_unit[i] = 1.0/256.0;
            }
        }

        for (int id=0; id<4; ++id)
        {
            for (int i=0; i<24; ++i)
            {
                pressure_info_.sensor[id].center[i] = ts_[id]->getCenter(i);
                pressure_info_.sensor[id].halfside1[i] = ts_[id]->getHalfside1(i);
                pressure_info_.sensor[id].halfside2[i] = ts_[id]->getHalfside2(i);
            }
        }
        port_tactile_info_out_.setDataSample(pressure_info_);
        port_tactile_out_.setDataSample(tactile_out_);
        port_max_pressure_out_.setDataSample(max_pressure_out_);
    }

    BarrettTactileSim::~BarrettTactileSim() {
    }

    bool BarrettTactileSim::configureHook() {
        Logger::In in("BarrettTactileSim::configureHook");

        if (prefix_.empty()) {
            Logger::log() << Logger::Error << "param 'prefix' is empty" << Logger::endl;
            return false;
        }

        return true;
    }

    void BarrettTactileSim::updateHook() {
        port_max_pressure_out_.write(max_pressure_out_);
        port_tactile_out_.write(tactile_out_);
        port_tactile_info_out_.write(pressure_info_);
    }

    bool BarrettTactileSim::startHook() {
      return true;
    }

ORO_LIST_COMPONENT_TYPE(BarrettTactileSim)

