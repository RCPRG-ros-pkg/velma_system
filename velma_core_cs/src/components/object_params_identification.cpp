// Copyright (c) 2020, Robot Control and Pattern Recognition Group,
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
// Author: Szymon Jarocki
//

#include "rtt/RTT.hpp"
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

#include <geometry_msgs/Wrench.h>
#include <math.h> 

using namespace RTT;

class ObjectParamsIdentification: public RTT::TaskContext
{
public:
    explicit ObjectParamsIdentification(const std::string &name);

    bool configureHook();
    bool startHook();
    void stopHook();
    void updateHook();

private:
    void calculateAndSendWeightOfTheObject(geometry_msgs::Wrench first_wrench_, geometry_msgs::Wrench second_wrench_);

    RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_right_slow_filtered_wrench_;  
    RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_left_slow_filtered_wrench_;      
    // RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_right_fast_filtered_wrench_;
    // RTT::InputPort<geometry_msgs::Wrench> port_current_sensor_left_fast_filtered_wrench_;

    RTT::InputPort<unsigned int> port_identification_measurement_right_command_;  
    RTT::InputPort<unsigned int> port_identification_measurement_left_command_;
    RTT::OutputPort<double> port_weight_of_the_object_;        

    geometry_msgs::Wrench current_sensor_right_slow_filtered_wrench_;
    geometry_msgs::Wrench current_sensor_left_slow_filtered_wrench_;    
    // geometry_msgs::Wrench current_sensor_right_fast_filtered_wrench_;
    // geometry_msgs::Wrench current_sensor_left_fast_filtered_wrench_;

    unsigned int identification_measurement_right_command_;
    unsigned int identification_measurement_left_command_;
    double weight_of_the_object_;

    geometry_msgs::Wrench first_right_sensor_identification_wrench_, second_right_sensor_identification_wrench_;
    geometry_msgs::Wrench first_left_sensor_identification_wrench_, second_left_sensor_identification_wrench_;    
    int command_index_right_, command_index_left_;
};

ObjectParamsIdentification::ObjectParamsIdentification(const std::string &name)
    : TaskContext(name, PreOperational)
    , port_current_sensor_right_slow_filtered_wrench_("CurrentSensorRightSlowFilteredWrench_INPORT")
    , port_current_sensor_left_slow_filtered_wrench_("CurrentSensorLeftSlowFilteredWrench_INPORT")    
    // , port_current_sensor_right_fast_filtered_wrench_("CurrentSensorRightFastFilteredWrench_INPORT")
    // , port_current_sensor_left_fast_filtered_wrench_("CurrentSensorLeftFastFilteredWrench_INPORT")
    , port_identification_measurement_right_command_("IdentificationMeasurementRightCommand_INPORT")
    , port_identification_measurement_left_command_("IdentificationMeasurementLeftCommand_INPORT")
    , port_weight_of_the_object_("WeightOfTheObject_OUTPORT")    
{
    this->ports()->addPort(port_current_sensor_right_slow_filtered_wrench_);
    this->ports()->addPort(port_current_sensor_left_slow_filtered_wrench_);    
    // this->ports()->addPort(port_current_sensor_right_fast_filtered_wrench_);
    // this->ports()->addPort(port_current_sensor_left_fast_filtered_wrench_);    
    this->ports()->addPort(port_identification_measurement_right_command_);
    this->ports()->addPort(port_identification_measurement_left_command_); 
    this->ports()->addPort(port_weight_of_the_object_);      
}

bool ObjectParamsIdentification::configureHook()
{
    Logger::In in("ObjectParamsIdentification::configureHook");

    command_index_right_ = 0;
    command_index_left_ = 0;

    return true;
}

bool ObjectParamsIdentification::startHook()
{
    //
    return true;
}

void ObjectParamsIdentification::stopHook() 
{
    //  
}

void ObjectParamsIdentification::updateHook() 
{
    if (port_current_sensor_right_slow_filtered_wrench_.read(current_sensor_right_slow_filtered_wrench_) != RTT::NewData) 
    {
        RTT::Logger::In in("ObjectParamsIdentification::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " could not read port \'" << port_current_sensor_right_slow_filtered_wrench_.getName() << "\'" << Logger::endl;
        return;
    } 

    if (port_current_sensor_left_slow_filtered_wrench_.read(current_sensor_left_slow_filtered_wrench_) != RTT::NewData) 
    {
        RTT::Logger::In in("ObjectParamsIdentification::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " could not read port \'" << port_current_sensor_left_slow_filtered_wrench_.getName() << "\'" << Logger::endl;
        return;
    }

    if (port_identification_measurement_right_command_.read(identification_measurement_right_command_) == RTT::NewData) 
    {
        RTT::Logger::In in("ObjectParamsIdentification::updateHook");
        Logger::log() << "New data transfered to identification_measurement_right_command_ variable" << Logger::endl;
        command_index_right_++;

        if (command_index_right_ == 1)
        {
            first_right_sensor_identification_wrench_ = current_sensor_right_slow_filtered_wrench_;
        }
        else if (command_index_right_ == 2)
        {
            second_right_sensor_identification_wrench_ = current_sensor_right_slow_filtered_wrench_;
            calculateAndSendWeightOfTheObject(first_right_sensor_identification_wrench_, second_right_sensor_identification_wrench_);
            command_index_right_ = 0;
        }
    }

    if (port_identification_measurement_left_command_.read(identification_measurement_left_command_) == RTT::NewData) 
    {
        RTT::Logger::In in("ObjectParamsIdentification::updateHook");
        Logger::log() << "New data transfered to identification_measurement_left_command_ variable" << Logger::endl;
        command_index_right_++;

        if (command_index_left_ == 1)
        {
            first_left_sensor_identification_wrench_ = current_sensor_left_slow_filtered_wrench_;
        }
        else if (command_index_left_ == 2)
        {
            second_left_sensor_identification_wrench_ = current_sensor_left_slow_filtered_wrench_;
            calculateAndSendWeightOfTheObject(first_left_sensor_identification_wrench_, second_left_sensor_identification_wrench_);
            command_index_left_ = 0;
        }
    }

    // std::cout << "current_sensor_right_slow_filtered_wrench_.force.x: " << current_sensor_right_slow_filtered_wrench_.force.x << std::endl;
    // std::cout << "current_sensor_right_slow_filtered_wrench_.force.y: " << current_sensor_right_slow_filtered_wrench_.force.y << std::endl;
    // std::cout << "current_sensor_right_slow_filtered_wrench_.force.z: " << current_sensor_right_slow_filtered_wrench_.force.z << std::endl;
    // std::cout << "current_sensor_right_slow_filtered_wrench_.torque.x: " << current_sensor_right_slow_filtered_wrench_.torque.x << std::endl;
    // std::cout << "current_sensor_right_slow_filtered_wrench_.torque.y: " << current_sensor_right_slow_filtered_wrench_.torque.y << std::endl;
    // std::cout << "current_sensor_right_slow_filtered_wrench_.torque.z: " << current_sensor_right_slow_filtered_wrench_.torque.z << std::endl;

    // std::cout << "current_sensor_left_slow_filtered_wrench_.force.x: " << current_sensor_left_slow_filtered_wrench_.force.x << std::endl;
    // std::cout << "current_sensor_left_slow_filtered_wrench_.force.y: " << current_sensor_left_slow_filtered_wrench_.force.y << std::endl;
    // std::cout << "current_sensor_left_slow_filtered_wrench_.force.z: " << current_sensor_left_slow_filtered_wrench_.force.z << std::endl;
    // std::cout << "current_sensor_left_slow_filtered_wrench_.torque.x: " << current_sensor_left_slow_filtered_wrench_.torque.x << std::endl;
    // std::cout << "current_sensor_left_slow_filtered_wrench_.torque.y: " << current_sensor_left_slow_filtered_wrench_.torque.y << std::endl;
    // std::cout << "current_sensor_left_slow_filtered_wrench_.torque.z: " << current_sensor_left_slow_filtered_wrench_.torque.z << std::endl;    
}
void ObjectParamsIdentification::calculateAndSendWeightOfTheObject(geometry_msgs::Wrench first_wrench_, geometry_msgs::Wrench second_wrench_)
{
    weight_of_the_object_ = sqrt(pow(first_wrench_.force.x - second_wrench_.force.x, 2) + pow(first_wrench_.force.y - second_wrench_.force.y, 2) + pow(first_wrench_.force.z - second_wrench_.force.z, 2));
    std::cout << "weight_of_the_object_: " << weight_of_the_object_ << std::endl;

    port_weight_of_the_object_.write(weight_of_the_object_);
}

ORO_LIST_COMPONENT_TYPE(ObjectParamsIdentification)
