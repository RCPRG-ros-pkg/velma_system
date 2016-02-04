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

#include "ft_sensor_gazebo.h"

bool FtSensorGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

    if(model.get() == NULL) {
        std::cout << "FtSensorGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
        return false;
    }

    model_ = model;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FtSensorGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (joint_.get() == NULL) {
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    gazebo::physics::JointWrench wr;
    wr = joint_->GetForceTorque(0);
    KDL::Wrench wr_W = KDL::Wrench( KDL::Vector(wr.body2Force.x, wr.body2Force.y, wr.body2Force.z), KDL::Vector(wr.body2Torque.x, wr.body2Torque.y, wr.body2Torque.z) );
    KDL::Wrench wr_S = T_W_S_.Inverse() * wr_W;

    cartesianWrench_out_.force.x = -wr_S.force.x();
    cartesianWrench_out_.force.y = -wr_S.force.y();
    cartesianWrench_out_.force.z = -wr_S.force.z();
    cartesianWrench_out_.torque.x = -wr_S.torque.x();
    cartesianWrench_out_.torque.y = -wr_S.torque.y();
    cartesianWrench_out_.torque.z = -wr_S.torque.z();
}

