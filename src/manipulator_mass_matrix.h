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

#ifndef MANIPULATOR_MASS_MATRIX_H__
#define MANIPULATOR_MASS_MATRIX_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/physics/dart/DARTJoint.hh>

namespace manipulator_mass_matrix {

class Link {
public:
    Link(const std::string &name);
    void setInertia(double mass, const gazebo::math::Vector3 &cog, double IXX, double IXY, double IXZ, double IYY, double IYZ, double IZZ);
    const std::string &getName() const;
    void setJointAcceleration(double accel);
    void setJointName(const std::string &name);
    const std::string &getJointName() const;
    void updateLocalJacobian(const gazebo::math::Pose &childLinkToJoint, const gazebo::math::Vector3 &axis);
    void setParent(std::shared_ptr<Link > &parent);
    void setChild(std::shared_ptr<Link > &child);
    void setRelativePose(const gazebo::math::Pose &p);
    void updateMassMatrix();
    void setIndex(int index);
    int getIndex() const;
    void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);

    gazebo::physics::ModelPtr model_;

protected:
    Eigen::Matrix<double, 6, 6> mI_;
    std::string name_;
    std::string joint_name_;
    double joint_accel_;
    Eigen::Matrix<double, 6, 1> mM_dV_;
    Eigen::Matrix<double, 6, 1> mM_F_;
    Eigen::Matrix<double, 6, 1> mJacobian_;
    std::shared_ptr<Link > parent_;
    std::shared_ptr<Link > child_;
    gazebo::math::Pose relative_pose_;
    int index_;

    
};

class Manipulator {
public:
    Manipulator(gazebo::physics::ModelPtr model, const std::string &first_joint, const std::string &last_joint,
                            double tool_mass, const gazebo::math::Vector3 &tool_cog, double tool_IXX, double tool_IXY,
                            double tool_IXZ, double tool_IYY, double tool_IYZ, double tool_IZZ);

    const Eigen::MatrixXd& getMassMatrix();
    void updatePoses(gazebo::physics::ModelPtr model);

protected:
    void setAccelerations(const Eigen::VectorXd &acc);

    typedef std::shared_ptr<Link > LinkPtr;
    typedef std::list<LinkPtr > LinkList;
    typedef std::vector<LinkPtr > LinkVec;
    LinkVec links_;
    Eigen::MatrixXd mM_;
};

/*
void BodyNode::_updateSpatialInertia()
{
  // G = | I - m*[r]*[r]   m*[r] |
  //     |        -m*[r]     m*I |

  // m*r
  double mr0 = mMass * mCenterOfMass[0];
  double mr1 = mMass * mCenterOfMass[1];
  double mr2 = mMass * mCenterOfMass[2];

  // m*[r]*[r]
  double mr0r0 = mr0 * mCenterOfMass[0];
  double mr1r1 = mr1 * mCenterOfMass[1];
  double mr2r2 = mr2 * mCenterOfMass[2];
  double mr0r1 = mr0 * mCenterOfMass[1];
  double mr1r2 = mr1 * mCenterOfMass[2];
  double mr2r0 = mr2 * mCenterOfMass[0];

  // Top left corner (3x3)
  mI(0, 0) =  mIxx + mr1r1 + mr2r2;
  mI(1, 1) =  mIyy + mr2r2 + mr0r0;
  mI(2, 2) =  mIzz + mr0r0 + mr1r1;
  mI(0, 1) =  mIxy - mr0r1;
  mI(0, 2) =  mIxz - mr2r0;
  mI(1, 2) =  mIyz - mr1r2;

  // Top right corner (3x3)
  mI(1, 5) = -mr0;
  mI(0, 5) =  mr1;
  mI(0, 4) = -mr2;
  mI(2, 4) =  mr0;
  mI(2, 3) = -mr1;
  mI(1, 3) =  mr2;
  assert(mI(0, 3) == 0.0);
  assert(mI(1, 4) == 0.0);
  assert(mI(2, 5) == 0.0);

  // Bottom right corner (3x3)
  mI(3, 3) =  mMass;
  mI(4, 4) =  mMass;
  mI(5, 5) =  mMass;
  assert(mI(3, 4) == 0.0);
  assert(mI(3, 5) == 0.0);
  assert(mI(4, 5) == 0.0);

  mI.triangularView<Eigen::StrictlyLower>() = mI.transpose();
}

void Skeleton::updateMassMatrix()
{
  if (getNumDofs() == 0)
    return;

  assert(static_cast<size_t>(mM.cols()) == getNumDofs()
         && static_cast<size_t>(mM.rows()) == getNumDofs());

  mM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  size_t dof = getNumDofs();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (size_t j = 0; j < dof; ++j)
  {
    e[j] = 1.0;
    setAccelerations(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateMassMatrix(&mM, j);
      size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        size_t iStart = (*it)->mParentJoint->getIndexInSkeleton(0);

        if (iStart + localDof < j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mM.triangularView<Eigen::StrictlyUpper>() = mM.transpose();

  // Restore the origianl generalized accelerations
  setAccelerations(originalGenAcceleration);

  mIsMassMatrixDirty = false;
}

// re = Inv(T)*s*T
Eigen::Vector6d AdInvT(const Eigen::Isometry3d& _T, const Eigen::Vector6d& _V) {
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear().transpose() * _V.head<3>();
  res.tail<3>().noalias() =
      _T.linear().transpose()
      * (_V.tail<3>() + _V.head<3>().cross(_T.translation()));
  return res;
}

void BodyNode::updateMassMatrix()
{
  mM_dV.setZero();
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    mM_dV.noalias() += mParentJoint->getLocalJacobian()
                       * mParentJoint->getAccelerations();
    assert(!math::isNan(mM_dV));
  }
  if (mParentBodyNode)
    mM_dV += math::AdInvT(mParentJoint->getLocalTransform(),
                          mParentBodyNode->mM_dV);
  assert(!math::isNan(mM_dV));
}

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

Eigen::Vector6d AdTAngular(const Eigen::Isometry3d& _T,
                           const Eigen::Vector3d& _w) {
  //--------------------------------------------------------------------------
  // w' = R*w
  // v' = p x R*w
  //--------------------------------------------------------------------------
  Eigen::Vector6d res;
  res.head<3>().noalias() = _T.linear() * _w;
  res.tail<3>() = _T.translation().cross(res.head<3>());
  return res;
}

void RevoluteJoint::updateLocalJacobian()
{
  // TODO(JS): This should be updated when mT_ChildBodyToJoint or mAxis.
  mJacobian = math::AdTAngular(mT_ChildBodyToJoint, mAxis);

  // Verification
  assert(!math::isNan(mJacobian));
}

Eigen::Vector6d dAdInvT(const Eigen::Isometry3d& _T,
                        const Eigen::Vector6d& _F) {
  Eigen::Vector6d res;
  res.tail<3>().noalias() = _T.linear() * _F.tail<3>();
  res.head<3>().noalias() = _T.linear() * _F.head<3>();
  res.head<3>() += _T.translation().cross(res.tail<3>());
  return res;
}

void BodyNode::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  //
  mM_F.noalias() = mI * mM_dV;

  // Verification
  assert(!math::isNan(mM_F));

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
//    mM_F += (*it)->getParentJoint()->getLocalTransform().Inverse() * (*it)->mM_F;
  }

  // Verification
  assert(!math::isNan(mM_F));

  //
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _MCol->block(iStart, _col, dof, 1).noalias() =
        mParentJoint->getLocalJacobian().transpose() * mM_F;
  }
}
*/
}   // namespace manipulator_mass_matrix

#endif  // MANIPULATOR_MASS_MATRIX
