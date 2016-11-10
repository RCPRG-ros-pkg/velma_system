/*
 Copyright (c) 2016, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include "manipulator_mass_matrix.h"

namespace manipulator_mass_matrix {

static Eigen::Vector3d ConvVec3(const gazebo::math::Vector3 &_vec3) {
    return Eigen::Vector3d(_vec3.x, _vec3.y, _vec3.z);
}

static gazebo::math::Vector3 ConvVec3(const Eigen::Vector3d &_vec3)
{
    return gazebo::math::Vector3(_vec3.x(), _vec3.y(), _vec3.z());
}

static Eigen::Quaterniond ConvQuat(const gazebo::math::Quaternion &_quat) {
    return Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z);
}

static gazebo::math::Quaternion ConvQuat(const Eigen::Quaterniond &_quat)
{
    return gazebo::math::Quaternion(_quat.w(), _quat.x(), _quat.y(), _quat.z());
}

static Eigen::Isometry3d ConvPose(const gazebo::math::Pose &_pose) {
// Below line doesn't work with 'libeigen3-dev is 3.0.5-1'
// return Eigen::Translation3d(ConvVec3(_pose.pos)) *
//        ConvQuat(_pose.rot);
    Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
    res.translation() = ConvVec3(_pose.pos);
    res.linear() = Eigen::Matrix3d(ConvQuat(_pose.rot));
    return res;
}

static gazebo::math::Pose ConvPose(const Eigen::Isometry3d &_T)
{
    gazebo::math::Pose pose;
    pose.pos = ConvVec3(_T.translation());
    pose.rot = ConvQuat(Eigen::Quaterniond(_T.linear()));
    return pose;
}

Manipulator::Manipulator(gazebo::physics::ModelPtr model, const std::string &first_joint, const std::string &last_joint,
                            double tool_mass, const gazebo::math::Vector3 &tool_cog, double tool_IXX, double tool_IXY,
                            double tool_IXZ, double tool_IYY, double tool_IYZ, double tool_IZZ) {
    gazebo::physics::Joint_V js = model->GetJoints();
    gazebo::physics::JointPtr joint = model->GetJoint(last_joint);

    LinkList links;

    links.push_front(LinkPtr(new Link(joint->GetChild()->GetName(), model->GetLink(joint->GetChild()->GetName()) )));
    links.front()->setInertia(tool_mass, tool_cog, tool_IXX, tool_IXY,
                            tool_IXZ, tool_IYY, tool_IYZ, tool_IZZ);
    links.front()->setJointName(joint->GetName());
    links.front()->model_ = model;

    while (joint && joint->GetName() != first_joint) {
        gazebo::physics::LinkPtr link = joint->GetParent();
//        std::cout << "Manipulator::Manipulator  " << joint->GetName() << std::endl;
        links.push_front(LinkPtr(new Link(link->GetName(), link)));
        gazebo::physics::InertialPtr in = link->GetInertial();
        links.front()->setInertia(in->GetMass(), in->GetCoG(), in->GetIXX(), in->GetIXY(),
                            in->GetIXZ(), in->GetIYY(), in->GetIYZ(), in->GetIZZ());

        for (gazebo::physics::Joint_V::const_iterator it = js.begin(); it != js.end(); it++) {
            if ( (*it)->GetChild()->GetName() == link->GetName() ) {
                joint = (*it);
                break;
            }
        }
        links.front()->setJointName(joint->GetName());
        links.front()->model_ = model;
    }
    links_.reserve(links.size());
    for (LinkList::const_iterator it = links.begin(); it != links.end(); it++) {
//        std::cout << "Manipulator::Manipulator  " << (*it)->getName() << "  " << (*it)->getJointName() << std::endl;
        links_.push_back( *it );
        joint = model->GetJoint((*it)->getJointName());
        (*it)->updateLocalJacobian(joint->GetInitialAnchorPose(), joint->GetAxisFrameOffset(0) * joint->GetLocalAxis(0));
    }

    for (int i = 1; i < links_.size(); i++) {
        links_[i]->setParent(links_[i-1]);
        links_[i-1]->setChild(links_[i]);
    }

    for (int i = 0; i < links_.size(); i++) {
        links_[i]->setIndex(i);
    }

    mM_ = Eigen::MatrixXd::Zero(links_.size(), links_.size());
}

void Manipulator::updatePoses(gazebo::physics::ModelPtr model) {
    links_[0]->setRelativePose( gazebo::math::Pose() );
    for (int i=1; i<links_.size(); ++i) {
        Eigen::Isometry3d T_W_1 = ConvPose(links_[i-1]->getGazeboLink()->GetWorldPose());
        Eigen::Isometry3d T_W_2 = ConvPose(links_[i]->getGazeboLink()->GetWorldPose());
        Eigen::Isometry3d T_1_2 = T_W_1.inverse() * T_W_2;
//        gazebo::math::Pose T_W_1 = model->GetLink( links_[i-1]->getName() )->GetWorldPose();
//        gazebo::math::Pose T_W_2 = model->GetLink( links_[i]->getName() )->GetWorldPose();
//        gazebo::math::Pose T_1_2 = T_W_1.GetInverse() * T_W_2;
        links_[i]->setRelativePose( ConvPose(T_1_2) );
    }
}

gazebo::physics::LinkPtr Link::getGazeboLink() const {
    return gz_link_;
}

void Link::setRelativePose(const gazebo::math::Pose &p) {
    relative_pose_ = p;
}

Link::Link(const std::string &name, const gazebo::physics::LinkPtr &gz_link) :
    name_(name),
    gz_link_(gz_link)
{
}

void Link::setIndex(int index) {
    index_ = index;
}

int Link::getIndex() const {
    return index_;
}

const std::string &Link::getName() const {
    return name_;
}

void Link::setJointName(const std::string &name) {
    joint_name_ = name;
}

const std::string &Link::getJointName() const {
    return joint_name_;
}

void Link::setParent(std::shared_ptr<Link > &parent) {
    parent_ = parent;
}

void Link::setChild(std::shared_ptr<Link > &child) {
    child_ = child;
}

void Link::setInertia(double mass, const gazebo::math::Vector3 &cog, double IXX, double IXY, double IXZ, double IYY, double IYZ, double IZZ) {
//void BodyNode::_updateSpatialInertia()
  // G = | I - m*[r]*[r]   m*[r] |
  //     |        -m*[r]     m*I |

  // m*r
  double mr0 = mass * cog.x;
  double mr1 = mass * cog.y;
  double mr2 = mass * cog.z;

  // m*[r]*[r]
  double mr0r0 = mr0 * cog.x;
  double mr1r1 = mr1 * cog.y;
  double mr2r2 = mr2 * cog.z;
  double mr0r1 = mr0 * cog.y;
  double mr1r2 = mr1 * cog.z;
  double mr2r0 = mr2 * cog.x;

  // Top left corner (3x3)
  mI_(0, 0) =  IXX + mr1r1 + mr2r2;
  mI_(1, 1) =  IYY + mr2r2 + mr0r0;
  mI_(2, 2) =  IZZ + mr0r0 + mr1r1;
  mI_(0, 1) =  IXY - mr0r1;
  mI_(0, 2) =  IXZ - mr2r0;
  mI_(1, 2) =  IYZ - mr1r2;

  // Top right corner (3x3)
  mI_(1, 5) = -mr0;
  mI_(0, 5) =  mr1;
  mI_(0, 4) = -mr2;
  mI_(2, 4) =  mr0;
  mI_(2, 3) = -mr1;
  mI_(1, 3) =  mr2;
  mI_(0, 3) = 0.0;
  mI_(1, 4) = 0.0;
  mI_(2, 5) = 0.0;

  // Bottom right corner (3x3)
  mI_(3, 3) =  mass;
  mI_(4, 4) =  mass;
  mI_(5, 5) =  mass;
  mI_(3, 4) = 0.0;
  mI_(3, 5) = 0.0;
  mI_(4, 5) = 0.0;

  mI_.triangularView<Eigen::StrictlyLower>() = mI_.transpose();
}

void Link::setJointAcceleration(double accel) {
    joint_accel_ = accel;
}

void Manipulator::setAccelerations(const Eigen::VectorXd &acc) {
    for (size_t j = 0; j < links_.size(); ++j) {
        links_[j]->setJointAcceleration( acc(j) );
    }
}

const Eigen::MatrixXd& Manipulator::getMassMatrix() {
//void Skeleton::updateMassMatrix()

  mM_.setZero();

  size_t dof = links_.size();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (size_t j = 0; j < dof; ++j) {
    e[j] = 1.0;
    setAccelerations(e);

    // Prepare cache data
    for (size_t i = 0; i < dof; ++i) {
        links_[i]->updateMassMatrix();
    }

    // Mass matrix
    for (int i = dof-1; i >= 0; --i) {
        links_[i]->aggregateMassMatrix(&mM_, j);
        size_t iStart = links_[i]->getIndex();

      if (iStart + 1 < j)
        break;
    }

    e[j] = 0.0;
  }
  mM_.triangularView<Eigen::StrictlyUpper>() = mM_.transpose();

    return mM_;
}

// re = Inv(T)*s*T
Eigen::Matrix<double, 6, 1> AdInvT(const Eigen::Isometry3d& _T, const Eigen::Matrix<double, 6, 1>& _V) {
  Eigen::Matrix<double, 6, 1> res;
  res.head<3>().noalias() = _T.linear().transpose() * _V.head<3>();
  res.tail<3>().noalias() =
      _T.linear().transpose()
      * (_V.tail<3>() + _V.head<3>().cross(_T.translation()));
  return res;
}

void Link::updateMassMatrix()
{
    mM_dV_.setZero();
    mM_dV_.noalias() += mJacobian_ * joint_accel_;
    if (parent_) {

//        gazebo::physics::JointPtr joint = model_->GetJoint(joint_name_);
//        gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
//        Eigen::Isometry3d i2 = joint_dart->GetDARTJoint()->getLocalTransform();

        mM_dV_ += AdInvT(ConvPose(relative_pose_), parent_->mM_dV_);
//        mM_dV_ += AdInvT(i2, parent_->mM_dV_);
    }
}

/*
// I + sin(t) / t*[S] + (1 - cos(t)) / t^2*[S]^2, where t = |S|
Eigen::Isometry3d expAngular(const Eigen::Vector3d& _s) {
  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
  double s2[] = { _s[0]*_s[0], _s[1]*_s[1], _s[2]*_s[2] };
  double s3[] = { _s[0]*_s[1], _s[1]*_s[2], _s[2]*_s[0] };
  double theta = sqrt(s2[0] + s2[1] + s2[2]);
  double cos_t = cos(theta);
  double alpha = 0.0;
  double beta = 0.0;

  if (theta > DART_EPSILON) {
    alpha = sin(theta) / theta;
    beta = (1.0 - cos_t) / theta / theta;
  } else {
    alpha = 1.0 - theta*theta/6.0;
    beta = 0.5 - theta*theta/24.0;
  }

  ret(0, 0) = beta*s2[0] + cos_t;
  ret(1, 0) = beta*s3[0] + alpha*_s[2];
  ret(2, 0) = beta*s3[2] - alpha*_s[1];

  ret(0, 1) = beta*s3[0] - alpha*_s[2];
  ret(1, 1) = beta*s2[1] + cos_t;
  ret(2, 1) = beta*s3[1] + alpha*_s[0];

  ret(0, 2) = beta*s3[2] + alpha*_s[1];
  ret(1, 2) = beta*s3[1] - alpha*_s[0];
  ret(2, 2) = beta*s2[2] + cos_t;

  return ret;
}

void RevoluteJoint::updateLocalTransform()
{
  mT = mT_ParentBodyToJoint
       * math::expAngular(mAxis * mPosition)
       * mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}
*/
Eigen::Matrix<double, 6, 1> AdTAngular(const Eigen::Isometry3d& _T,
                           const Eigen::Vector3d& _w) {
  //--------------------------------------------------------------------------
  // w' = R*w
  // v' = p x R*w
  //--------------------------------------------------------------------------
  Eigen::Matrix<double, 6, 1> res;
  res.head<3>().noalias() = _T.linear() * _w;
  res.tail<3>() = _T.translation().cross(res.head<3>());
  return res;
}

void Link::updateLocalJacobian(const gazebo::math::Pose &childLinkToJoint, const gazebo::math::Vector3 &axis) {

//    gazebo::physics::JointPtr joint = model_->GetJoint(joint_name_);
//    gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
//    dart::dynamics::RevoluteJoint *jnt = dynamic_cast<dart::dynamics::RevoluteJoint* >(joint_dart->GetDARTJoint());

//    Eigen::Vector3d ax = jnt->getAxis();
//    std::cout << "axis: " << axis.x << " " << axis.y << " " << axis.z << "            " << ax.transpose() << std::endl;
//    std::cout << "pose: " << ConvPose(childLinkToJoint).translation() << " " << ConvPose(childLinkToJoint).linear() << std::endl
//            << "*** " << jnt->getTransformFromChildBodyNode().translation() << " " << jnt->getTransformFromChildBodyNode().linear() << std::endl;

    mJacobian_ = AdTAngular(ConvPose(childLinkToJoint), ConvVec3(axis));
//    mJacobian_ = AdTAngular(jnt->getTransformFromChildBodyNode(), ax);
}


Eigen::Matrix<double, 6, 1> dAdInvT(const Eigen::Isometry3d& _T,
                        const Eigen::Matrix<double, 6, 1>& _F) {
  Eigen::Matrix<double, 6, 1> res;
  res.tail<3>().noalias() = _T.linear() * _F.tail<3>();
  res.head<3>().noalias() = _T.linear() * _F.head<3>();
  res.head<3>() += _T.translation().cross(res.tail<3>());
  return res;
}

void Link::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  //
  mM_F_.noalias() = mI_ * mM_dV_;

  // Verification
//  assert(!math::isNan(mM_F_));

  //
    if (child_) {

//        gazebo::physics::JointPtr joint = model_->GetJoint(child_->joint_name_);
//        gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
        

//        Eigen::Isometry3d i1 = ConvPose(child_->relative_pose_);
//        Eigen::Isometry3d i2 = joint_dart->GetDARTJoint()->getLocalTransform();
//        std::cout << "pose: " << i1.translation() << " " << i1.linear() << std::endl
//                    << "*** " << i2.translation() << " " << i2.linear() << std::endl;

        mM_F_ += dAdInvT(ConvPose(child_->relative_pose_),
                            child_->mM_F_);
//        mM_F_ += dAdInvT(i2,
//                            child_->mM_F_);
    }

  // Verification
//  assert(!math::isNan(mM_F));

  //
//  int dof = mParentJoint->getNumDofs();
//  if (dof > 0)
//  {
    int iStart = index_;//mParentJoint->getIndexInSkeleton(0);
//    _MCol->block(iStart, _col, dof, 1).noalias() =
//        mParentJoint->getLocalJacobian().transpose() * mM_F_;
    _MCol->block(iStart, _col, 1, 1).noalias() =
        mJacobian_.transpose() * mM_F_;
//  }
}


}   // namespace manipulator_mass_matrix

