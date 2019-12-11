// Copyright (c) 2019, Robot Control and Pattern Recognition Group,
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
// Authors: Szymon Jarocki, Dawid Seredyński
//

#include <rtt/Component.hpp>

#include "data_buffer_manager.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Int32.h>

using namespace RTT;
			
class DataBufferManager: public RTT::TaskContext
{
public:
    explicit DataBufferManager(const std::string &name);

    bool configureHook();
    bool startHook();
    void stopHook();
    void updateHook();

private:
	typedef Eigen::Matrix<double, 15, 1>  VectorD;
	typedef Eigen::Matrix<double, 7, 1>  Vector7;

	DataBufferManagerSupport<VectorD> joint_position_data_;
	DataBufferManagerSupport<VectorD> joint_position_command_data_;	
	DataBufferManagerSupport<VectorD> joint_velocity_data_; 
	DataBufferManagerSupport<Vector7> joint_torque_rArm_data_;
	DataBufferManagerSupport<Vector7> joint_torque_lArm_data_;
	DataBufferManagerSupport<double> joint_torque_torso_command_data_;
	DataBufferManagerSupport<Vector7> joint_torque_rArm_command_data_;
	DataBufferManagerSupport<Vector7> joint_torque_lArm_command_data_;
	DataBufferManagerSupport<geometry_msgs::Pose> cartesian_position_rEffector_data_;
	DataBufferManagerSupport<geometry_msgs::Pose> cartesian_position_lEffector_data_;
	DataBufferManagerSupport<geometry_msgs::Pose> cartesian_position_rEffector_command_data_;
	DataBufferManagerSupport<geometry_msgs::Pose> cartesian_position_lEffector_command_data_;

	RTT::InputPort<std_msgs::Int32> port_write_data_command_;
	std_msgs::Int32 write_data_command_;

	int buffer_length;
	int file_number;

	void writeVectorD(std::ofstream &logfile, VectorD data, bool data_valid)
	{
		for (int j = 0; j < 15; ++j)
		{
			if (data_valid)
			{
				logfile << data[j] << " ";
			}
			else
			{
				logfile << "X ";
			}
		}
	}

	void writeVector7(std::ofstream &logfile, Vector7 data, bool data_valid)
	{
		for (int j = 0; j < 7; ++j)
		{
			if (data_valid)
			{
				logfile << data[j] << " ";
			}
			else
			{
				logfile << "X ";
			}
		}
	}

	void writeDouble(std::ofstream &logfile, double data, bool data_valid)
	{
		if (data_valid)
		{
			logfile << data << " ";
		}
		else
		{
			logfile << "X ";
		}
	}

	void writeGeometryMsgsPose(std::ofstream &logfile, geometry_msgs::Pose data, bool data_valid)
	{
		if (data_valid)
		{
			logfile << data.position.x << " ";
			logfile << data.position.y << " ";
			logfile << data.position.z << " ";
			logfile << data.orientation.x << " ";
			logfile << data.orientation.y << " ";
			logfile << data.orientation.z << " ";
			logfile << data.orientation.w << " ";			
		}
		else
		{
			logfile << "X X X X X X X ";
		}
	}
};

DataBufferManager::DataBufferManager(const std::string &name)
	: TaskContext(name, PreOperational)
	, joint_position_data_(*this, "JointPosition_INPORT")
	, joint_position_command_data_(*this, "JointPositionCommand_INPORT")
	, joint_velocity_data_(*this, "JointVelocity_INPORT")
	, joint_torque_rArm_data_(*this, "RightArmTorque_INPORT")
	, joint_torque_lArm_data_(*this, "LeftArmTorque_INPORT")
	, joint_torque_torso_command_data_(*this, "TorsoTorqueCommand_INPORT")
	, joint_torque_rArm_command_data_(*this, "RightArmTorqueCommand_INPORT")
	, joint_torque_lArm_command_data_(*this, "LeftArmTorqueCommand_INPORT")
	, cartesian_position_rEffector_data_(*this,	"RightEffectorCartesianPosition_INPORT")
	, cartesian_position_lEffector_data_(*this, "LeftEffectorCartesianPosition_INPORT")	
	, cartesian_position_rEffector_command_data_(*this, "RightEffectorCartesianPositionCommand_INPORT")
	, cartesian_position_lEffector_command_data_(*this, "LeftEffectorCartesianPositionCommand_INPORT")
	, port_write_data_command_("WriteDataCommand_INPORT")
{	
	this->ports()->addPort(port_write_data_command_);	
}

bool DataBufferManager::configureHook()
{
	file_number = 0;
	return true;
}


bool DataBufferManager::startHook()
{
	//
	return true;
}

void DataBufferManager::stopHook() 
{
	//
}

void DataBufferManager::updateHook()
{
	joint_position_data_.port_read();
	joint_position_command_data_.port_read();		
	joint_velocity_data_.port_read();
	joint_torque_rArm_data_.port_read();
	joint_torque_lArm_data_.port_read();
	joint_torque_torso_command_data_.port_read();
	joint_torque_rArm_command_data_.port_read();
	joint_torque_lArm_command_data_.port_read();
	cartesian_position_rEffector_data_.port_read();
	cartesian_position_lEffector_data_.port_read();
	cartesian_position_rEffector_command_data_.port_read();
	cartesian_position_lEffector_command_data_.port_read();

	if (port_write_data_command_.read(write_data_command_) == RTT::NewData)
	{
		std::ofstream logfile;
		std::ostringstream ss;

		file_number++;
		ss << "/tmp/velma_core_cs_data_" << file_number <<".txt";

		logfile.open(ss.str());
		std::cout << "writing data from velma_core_cs subsystem" << std::endl;

		buffer_length = joint_position_data_.getSize();
		for (int i = 0; i < buffer_length; ++i) 
		{
			// if (i%1000 == 0)
			// {
			// 	std::cout << "data log " << i << "/" << buffer_length-1 << std::endl;				
			// }

			VectorD joint_position = joint_position_data_.getData(i);
			bool joint_position_valid = joint_position_data_.isValid(i);
			writeVectorD(logfile, joint_position, joint_position_valid);

			VectorD joint_position_command = joint_position_command_data_.getData(i);
			bool joint_position_command_valid = joint_position_command_data_.isValid(i);
			writeVectorD(logfile, joint_position_command, joint_position_command_valid);			

			VectorD joint_velocity = joint_velocity_data_.getData(i);
			bool joint_velocity_valid = joint_velocity_data_.isValid(i);
			writeVectorD(logfile, joint_velocity, joint_velocity_valid);

			Vector7 joint_torque_rArm = joint_torque_rArm_data_.getData(i);
			bool joint_torque_rArm_valid = joint_torque_rArm_data_.isValid(i);
			writeVector7(logfile, joint_torque_rArm, joint_torque_rArm_valid);

			Vector7 joint_torque_lArm = joint_torque_lArm_data_.getData(i);
			bool joint_torque_lArm_valid = joint_torque_lArm_data_.isValid(i);
			writeVector7(logfile, joint_torque_lArm, joint_torque_lArm_valid);

			double joint_torque_torso_command = joint_torque_torso_command_data_.getData(i);
			bool joint_torque_torso_command_valid = joint_torque_torso_command_data_.isValid(i);
			writeDouble(logfile, joint_torque_torso_command, joint_torque_torso_command_valid);

			Vector7 joint_torque_rArm_command = joint_torque_rArm_command_data_.getData(i);
			bool joint_torque_rArm_command_valid = joint_torque_rArm_command_data_.isValid(i);
			writeVector7(logfile, joint_torque_rArm_command, joint_torque_rArm_command_valid);

			Vector7 joint_torque_lArm_command = joint_torque_lArm_command_data_.getData(i);
			bool joint_torque_lArm_command_valid = joint_torque_lArm_command_data_.isValid(i);
			writeVector7(logfile, joint_torque_lArm_command, joint_torque_lArm_command_valid);			

			geometry_msgs::Pose cartesian_position_rEffector = cartesian_position_rEffector_data_.getData(i);
			bool cartesian_position_rEffector_valid = cartesian_position_rEffector_data_.isValid(i);
			writeGeometryMsgsPose(logfile, cartesian_position_rEffector, cartesian_position_rEffector_valid);

			geometry_msgs::Pose cartesian_position_lEffector = cartesian_position_lEffector_data_.getData(i);
			bool cartesian_position_lEffector_valid = cartesian_position_lEffector_data_.isValid(i);
			writeGeometryMsgsPose(logfile, cartesian_position_lEffector, cartesian_position_lEffector_valid);
			
			geometry_msgs::Pose cartesian_position_rEffector_command = cartesian_position_rEffector_command_data_.getData(i);
			bool cartesian_position_rEffector_command_valid = cartesian_position_rEffector_command_data_.isValid(i);
			writeGeometryMsgsPose(logfile, cartesian_position_rEffector_command, cartesian_position_rEffector_command_valid);

			geometry_msgs::Pose cartesian_position_lEffector_command = cartesian_position_lEffector_command_data_.getData(i);
			bool cartesian_position_lEffector_command_valid = cartesian_position_lEffector_command_data_.isValid(i);
			writeGeometryMsgsPose(logfile, cartesian_position_lEffector_command, cartesian_position_lEffector_command_valid);

			logfile << std::endl;
		}
		logfile.close();
		std::cout << "writing data log DONE" << std::endl;
	}
}

ORO_CREATE_COMPONENT(DataBufferManager)

