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

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <geometry_msgs/Wrench.h>

class OptoforceSim : public RTT::TaskContext
{
public:
    // public methods
    OptoforceSim(std::string const& name);
    ~OptoforceSim();
    void updateHook();
    bool startHook();
    bool configureHook();

  protected:

    // ROS parameters
    std::string device_name_;
    const int n_sensors_;
    std::vector<std::string > frame_id_vec_;

    int32_t median_filter_samples_, median_filter_max_samples_;

    // OROCOS ports
    boost::array<geometry_msgs::Wrench, 3 > force_out_;
    RTT::OutputPort<boost::array<geometry_msgs::Wrench, 3 > > port_force_out_;

    bool data_valid_;
};

using namespace RTT;

    OptoforceSim::OptoforceSim(std::string const& name) : 
        TaskContext(name, RTT::TaskContext::PreOperational),
        n_sensors_(3),
        data_valid_(false),
        port_force_out_("force_OUTPORT", false)
    {
        this->addProperty("device_name", device_name_);
        this->addProperty("frame_id_vec", frame_id_vec_);

        this->ports()->addPort(port_force_out_);
    }

    OptoforceSim::~OptoforceSim() {
    }

    bool OptoforceSim::configureHook() {
        Logger::In in("OptoforceSim::configureHook");

        return true;
    }

    void OptoforceSim::updateHook() {
        // TODO
        force_out_[0] = geometry_msgs::Wrench();
        force_out_[1] = geometry_msgs::Wrench();
        force_out_[2] = geometry_msgs::Wrench();
        port_force_out_.write(force_out_);

//        if (!data_valid_) {
//            return;
//        }
/*
        for (int i = 0; i < n_sensors_; i++) {
            port_force_out_[i]->write(force_out_[i]);
        }
*/
    }

    bool OptoforceSim::startHook() {
      return true;
    }


ORO_LIST_COMPONENT_TYPE(OptoforceSim)

