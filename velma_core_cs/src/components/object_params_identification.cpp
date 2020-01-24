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

#include <kdl/frames.hpp>
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
    typedef geometry_msgs::Wrench gW;
    void calculateAndSendParametersOfTheObject(gW first_wrench_, gW second_wrench_, gW third_wrench_, gW fourth_wrench_, std::string side_);

    RTT::InputPort<gW> port_current_sensor_right_slow_filtered_wrench_;  
    RTT::InputPort<gW> port_current_sensor_left_slow_filtered_wrench_;      
    // RTT::InputPort<gW> port_current_sensor_right_fast_filtered_wrench_;
    // RTT::InputPort<gW> port_current_sensor_left_fast_filtered_wrench_;

    RTT::InputPort<unsigned int> port_identification_measurement_right_command_;  
    RTT::InputPort<unsigned int> port_identification_measurement_left_command_;
    RTT::OutputPort<double> port_weight_of_the_object_;  
    RTT::OutputPort<KDL::Vector> port_center_of_mass_location_right_;      
    RTT::OutputPort<KDL::Vector> port_center_of_mass_location_left_; 

    gW current_sensor_right_slow_filtered_wrench_, current_sensor_left_slow_filtered_wrench_;    
    // gW current_sensor_right_fast_filtered_wrench_, current_sensor_left_fast_filtered_wrench_;

    unsigned int identification_measurement_right_command_, identification_measurement_left_command_;
    double weight_of_the_object_, sign_;
    KDL::Vector position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_;

    gW first_right_wrench_, second_right_wrench_, third_right_wrench_, fourth_right_wrench_;
    gW first_left_wrench_, second_left_wrench_, third_left_wrench_, fourth_left_wrench_;    
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
    , port_center_of_mass_location_right_("CenterOfMassLocationRight_OUTPORT") 
    , port_center_of_mass_location_left_("CenterOfMassLocationLeft_OUTPORT")       
{
    this->ports()->addPort(port_current_sensor_right_slow_filtered_wrench_);
    this->ports()->addPort(port_current_sensor_left_slow_filtered_wrench_);    
    // this->ports()->addPort(port_current_sensor_right_fast_filtered_wrench_);
    // this->ports()->addPort(port_current_sensor_left_fast_filtered_wrench_);    
    this->ports()->addPort(port_identification_measurement_right_command_);
    this->ports()->addPort(port_identification_measurement_left_command_); 
    this->ports()->addPort(port_weight_of_the_object_); 
    this->ports()->addPort(port_center_of_mass_location_right_);   
    this->ports()->addPort(port_center_of_mass_location_left_);        
}

bool ObjectParamsIdentification::configureHook()
{
    Logger::In in("ObjectParamsIdentification::configureHook");

    command_index_right_ = 1;
    command_index_left_ = 1;

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

        if (identification_measurement_right_command_ == 1 && command_index_right_ == 1)
        {
            first_right_wrench_ = current_sensor_right_slow_filtered_wrench_;
            command_index_right_ = 2;
        }
        else if (identification_measurement_right_command_ == 2 && command_index_right_ == 2)
        {
            second_right_wrench_ = current_sensor_right_slow_filtered_wrench_;
            command_index_right_ = 3;
        }
        else if (identification_measurement_right_command_ == 3 && command_index_right_ == 3)
        {
            third_right_wrench_ = current_sensor_right_slow_filtered_wrench_;
            command_index_right_ = 4;
        }
        else if (identification_measurement_right_command_ == 4 && command_index_right_ == 4)
        {
            fourth_right_wrench_ = current_sensor_right_slow_filtered_wrench_;
            calculateAndSendParametersOfTheObject(first_right_wrench_, second_right_wrench_, third_right_wrench_, fourth_right_wrench_, "right");
            command_index_right_ = 1;
        }
    }

    if (port_identification_measurement_left_command_.read(identification_measurement_left_command_) == RTT::NewData) 
    {
        RTT::Logger::In in("ObjectParamsIdentification::updateHook");
        Logger::log() << "New data transfered to identification_measurement_left_command_ variable" << Logger::endl;

        if (identification_measurement_left_command_ == 1 && command_index_left_ == 1)
        {
            first_left_wrench_ = current_sensor_left_slow_filtered_wrench_;
            command_index_left_ = 2;
        }
        else if (identification_measurement_left_command_ == 2 && command_index_left_ == 2)
        {
            second_left_wrench_ = current_sensor_left_slow_filtered_wrench_;
            command_index_left_ = 3;
        }
        else if (identification_measurement_left_command_ == 3 && command_index_left_ == 3)
        {
            third_left_wrench_ = current_sensor_left_slow_filtered_wrench_;
            command_index_left_ = 4;
        }
        else if (identification_measurement_left_command_ == 4 && command_index_left_ == 4)
        {
            fourth_left_wrench_ = current_sensor_left_slow_filtered_wrench_;
            calculateAndSendParametersOfTheObject(first_left_wrench_, second_left_wrench_, third_left_wrench_, fourth_left_wrench_, "left");
            command_index_left_ = 1;
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
void ObjectParamsIdentification::calculateAndSendParametersOfTheObject(gW first_wrench_, gW second_wrench_, gW third_wrench_, gW fourth_wrench_, std::string side_)
{
    //weight_of_the_object_ = sqrt(pow(first_wrench_.force.x - third_wrench_.force.x, 2) + pow(first_wrench_.force.y - third_wrench_.force.y, 2) + pow(first_wrench_.force.z - third_wrench_.force.z, 2));
    weight_of_the_object_ = first_wrench_.force.z - third_wrench_.force.z;
    std::cout << "weight_of_the_object_: " << weight_of_the_object_ << std::endl;

    // if (side_ == "right")
    // {
    //     sign_ = 1.0;
    // }
    // else if (side_ == "left")
    // {
    //     sign_ = -1.0; 
    // } 
    // position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[0] = sign_*(third_wrench_.torque.y - first_wrench_.torque.y)/weight_of_the_object_; // x_wrist_axis || z_sensor_axis
    // position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[1] = -sign_*(third_wrench_.torque.z - first_wrench_.torque.z)/weight_of_the_object_; // y_wrist_axis || y_sensor_axis; 
    // position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[2] = sign_*(fourth_wrench_.torque.z - second_wrench_.torque.z)/weight_of_the_object_; // z_wrist_axis || x_sensor_axis;

    position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[0] = (third_wrench_.torque.y - first_wrench_.torque.y)/weight_of_the_object_;
    position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[1] = -(third_wrench_.torque.x - first_wrench_.torque.x)/weight_of_the_object_;
    position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[2] = -(fourth_wrench_.torque.x - second_wrench_.torque.x)/weight_of_the_object_;
    std::cout << "position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_x: " << position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[0] << std::endl;
    std::cout << "position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_y: " << position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[1] << std::endl;
    std::cout << "position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_z: " << position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[2] << std::endl;

    port_weight_of_the_object_.write(weight_of_the_object_);

    if (side_ == "right")
    {
        port_center_of_mass_location_right_.write(position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_);        
    }
    else if (side_ == "left")
    {
        port_center_of_mass_location_left_.write(position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_);        
    }
}

ORO_LIST_COMPONENT_TYPE(ObjectParamsIdentification)
