#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Int32.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/dart/DARTModel.hh>
#include <gazebo/physics/dart/DARTJoint.hh>
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

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <kuka_lwr_fri/friComm.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;


class VelmaGazebo : public RTT::TaskContext
{
protected:
    ros::NodeHandle *nh_;

    gazebo::physics::DARTModelPtr model_dart_;
    dart::dynamics::Skeleton *dart_sk_;

    std::vector<gazebo::physics::JointPtr>  r_joints_;
    std::vector<gazebo::physics::JointPtr>  l_joints_;
    std::vector<gazebo::physics::JointPtr>  t_joints_;
    std::vector<int> r_indices_;
    std::vector<int> l_indices_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // right KUKA FRI ports
    RTT::InputPort<Eigen::VectorXd >        port_r_JointTorqueCommand_in_;  // FRIr.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_r_KRL_CMD_in_;             // FRIr.KRL_CMD
    RTT::OutputPort<tFriRobotState >        port_r_RobotState_out_;         // FRIr.RobotState
    RTT::OutputPort<tFriIntfState >         port_r_FRIState_out_;           // FRIr.FRIState
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointPosition_out_;      // FRIr.JointPosition
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointVelocity_out_;      // FRIr.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_r_CartesianWrench_out_;    // FRIr.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_r_MassMatrix_out_;         // FRIr.MassMatrix
    RTT::OutputPort<Eigen::VectorXd >       port_r_JointTorque_out_;        // FRIr.JointTorque
    RTT::OutputPort<Eigen::VectorXd >       port_r_GravityTorque_out_;      // FRIr.GravityTorque

    Eigen::VectorXd       r_JointTorqueCommand_in_;
    std_msgs::Int32       r_KRL_CMD_in_;
    tFriRobotState        r_RobotState_out_;
    tFriIntfState         r_FRIState_out_;
    Eigen::VectorXd       r_JointPosition_out_;
    Eigen::VectorXd       r_JointVelocity_out_;
    geometry_msgs::Wrench r_CartesianWrench_out_;
    Matrix77d             r_MassMatrix_out_;
    Eigen::VectorXd       r_JointTorque_out_;
    Eigen::VectorXd       r_GravityTorque_out_;

    // left KUKA FRI ports

    RTT::InputPort<Eigen::VectorXd >        port_l_JointTorqueCommand_in_;  // FRIl.JointTorqueCommand
    RTT::InputPort<std_msgs::Int32 >        port_l_KRL_CMD_in_;             // FRIl.KRL_CMD
    RTT::OutputPort<tFriRobotState >        port_l_RobotState_out_;         // FRIl.RobotState
    RTT::OutputPort<tFriIntfState >         port_l_FRIState_out_;           // FRIl.FRIState
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointPosition_out_;      // FRIl.JointPosition
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointVelocity_out_;      // FRIl.JointVelocity
    RTT::OutputPort<geometry_msgs::Wrench > port_l_CartesianWrench_out_;    // FRIl.CartesianWrench
    RTT::OutputPort<Matrix77d >             port_l_MassMatrix_out_;         // FRIl.MassMatrix
    RTT::OutputPort<Eigen::VectorXd >       port_l_JointTorque_out_;        // FRIl.JointTorque
    RTT::OutputPort<Eigen::VectorXd >       port_l_GravityTorque_out_;      // FRIl.GravityTorque

    Eigen::VectorXd       l_JointTorqueCommand_in_;
    std_msgs::Int32       l_KRL_CMD_in_;
    tFriRobotState        l_RobotState_out_;
    tFriIntfState         l_FRIState_out_;
    Eigen::VectorXd       l_JointPosition_out_;
    Eigen::VectorXd       l_JointVelocity_out_;
    geometry_msgs::Wrench l_CartesianWrench_out_;
    Matrix77d             l_MassMatrix_out_;
    Eigen::VectorXd       l_JointTorque_out_;
    Eigen::VectorXd       l_GravityTorque_out_;

    // torso ports
    RTT::InputPort<Eigen::VectorXd >    port_t_JointTorqueCommand_in_;  // TorsoTrqSplit.InputPort
    RTT::OutputPort<Eigen::VectorXd >   port_t_JointPosition_out_;      // TorsoPosAggregate.OutputPort
    RTT::OutputPort<Eigen::VectorXd >   port_t_JointVelocity_out_;      // TorsoVelAggregate.OutputPort

    Eigen::VectorXd t_JointTorqueCommand_in_;
    Eigen::VectorXd t_JointPosition_out_;
    Eigen::VectorXd t_JointVelocity_out_;

    VelmaGazebo(std::string const& name) : 
        TaskContext(name)
    {

        nh_ = new ros::NodeHandle();
        std::cout << "VelmaGazebo ROS node namespace: " << nh_->getNamespace() << std::endl;

        std::cout << "VelmaGazebo ROS node name: " << ros::this_node::getName() << std::endl;
        std::cout << "VelmaGazebo ROS node namespace2: " << ros::this_node::getNamespace() << std::endl;

//        addProperty("LWRlDiag/prefix", prop_prefix);
//        addProperty("parameter", prop_parameter);

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&VelmaGazebo::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&VelmaGazebo::gazeboUpdateHook,this,RTT::ClientThread);

        // right KUKA FRI ports
        this->ports()->addPort("r_JointTorqueCommand",        port_r_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("r_KRL_CMD",                   port_r_KRL_CMD_in_).doc("");
        this->ports()->addPort("r_CartesianWrench",           port_r_CartesianWrench_out_).doc("");
        this->ports()->addPort("r_RobotState",                port_r_RobotState_out_).doc("");
        this->ports()->addPort("r_FRIState",                  port_r_FRIState_out_).doc("");
        this->ports()->addPort("r_JointVelocity",             port_r_JointVelocity_out_).doc("");
        this->ports()->addPort("r_MassMatrix",                port_r_MassMatrix_out_).doc("");
        this->ports()->addPort("r_JointTorque",               port_r_JointTorque_out_).doc("");
        this->ports()->addPort("r_GravityTorque",             port_r_GravityTorque_out_);
        this->ports()->addPort("r_JointPosition",             port_r_JointPosition_out_).doc("");
        r_JointTorqueCommand_in_.resize(7);
        r_JointPosition_out_.resize(7);
        r_JointVelocity_out_.resize(7);
        r_JointTorque_out_.resize(7);
        r_GravityTorque_out_.resize(7);
        port_r_JointPosition_out_.setDataSample(    r_JointPosition_out_);
        port_r_JointVelocity_out_.setDataSample(    r_JointVelocity_out_);
        port_r_JointTorque_out_.setDataSample(      r_JointTorque_out_);
        port_r_GravityTorque_out_.setDataSample(    r_GravityTorque_out_);

        // left KUKA FRI ports
        this->ports()->addPort("l_JointTorqueCommand",        port_l_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("l_KRL_CMD",                   port_l_KRL_CMD_in_).doc("");
        this->ports()->addPort("l_CartesianWrench",           port_l_CartesianWrench_out_).doc("");
        this->ports()->addPort("l_RobotState",                port_l_RobotState_out_).doc("");
        this->ports()->addPort("l_FRIState",                  port_l_FRIState_out_).doc("");
        this->ports()->addPort("l_JointVelocity",             port_l_JointVelocity_out_).doc("");
        this->ports()->addPort("l_MassMatrix",                port_l_MassMatrix_out_).doc("");
        this->ports()->addPort("l_JointTorque",               port_l_JointTorque_out_).doc("");
        this->ports()->addPort("l_GravityTorque",             port_l_GravityTorque_out_);
        this->ports()->addPort("l_JointPosition",             port_l_JointPosition_out_).doc("");
        l_JointTorqueCommand_in_.resize(7);
        l_JointPosition_out_.resize(7);
        l_JointVelocity_out_.resize(7);
        l_JointTorque_out_.resize(7);
        l_GravityTorque_out_.resize(7);
        port_l_JointPosition_out_.setDataSample(    l_JointPosition_out_);
        port_l_JointVelocity_out_.setDataSample(    l_JointVelocity_out_);
        port_l_JointTorque_out_.setDataSample(      l_JointTorque_out_);
        port_l_GravityTorque_out_.setDataSample(    l_GravityTorque_out_);

        // torso ports
        this->ports()->addPort("t_JointTorqueCommand",        port_t_JointTorqueCommand_in_).doc("");
        this->ports()->addPort("t_JointPosition",             port_t_JointPosition_out_).doc("");
        this->ports()->addPort("t_JointVelocity",             port_t_JointVelocity_out_).doc("");
        t_JointTorqueCommand_in_.resize(1);
        t_JointPosition_out_.resize(1);
        t_JointVelocity_out_.resize(1);
    }

    ~VelmaGazebo() {
    }

    void updateHook() {
      // Synchronize with gazeboUpdate()
      RTT::os::MutexLock lock(gazebo_mutex_);
    }

    bool startHook() {
      return true;
    }

    bool configureHook() {
//        std::cout << "VelmaGazebo::configureHook: property: " << prop_prefix << std::endl;
//        std::cout << "VelmaGazebo::configureHook: property2: " << prop_parameter << std::endl;


        return true;
    }

    bool gazeboConfigureHook(gazebo::physics::ModelPtr model) {
        if(model.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();

        // fill in gazebo joints pointer vectors
        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("right_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->r_joints_.push_back(joint);
                r_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 7; i++) {
            std::string joint_name = std::string("left_arm_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);
            gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast < gazebo::physics::DARTJoint > ( joint );
            if (joint) {
                this->l_joints_.push_back(joint);
                l_indices_.push_back(joint_dart->GetDARTJoint()->getIndexInSkeleton(0));
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }

        for(unsigned int i = 0; i < 1; i++) {
            std::string joint_name = std::string("torso_") + std::to_string(i) + "_joint";
            gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
            if (joint) {
                this->t_joints_.push_back(joint);
            }
            else {
                RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
                return false;
            }
        }


/*      
      // get parameter name
    //  if (_sdf->HasElement("robotPrefix"))
    //    this->robotPrefix = _sdf->GetElement("robotPrefix")->GetValueString();
      chain_start = std::string("calib_") + this->robotPrefix_ + "_arm_base_link";
    //  if (_sdf->HasElement("baseLink"))
    //    this->chain_start = _sdf->GetElement("baseLink")->GetValueString();

      chain_end = this->robotPrefix_ + "_arm_7_link";
    //  if (_sdf->HasElement("toolLink"))
    //    this->chain_end = _sdf->GetElement("toolLink")->GetValueString();
      
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }
      this->rosnode_ = new ros::NodeHandle();
        
      GetRobotChain();
      
      jnt_pos_.resize(7);
      jnt_vel_.resize(7);

      jnt_pos_cmd_.resize(7);
      jnt_trq_cmd_.resize(7);
      
      for(unsigned int i = 0; i< 7; i++)
      {
        // fill in gazebo joints pointer
        std::string joint_name = this->robotPrefix_ + "_arm_" + (char)(i + 48) + "_joint";
        gazebo::physics::JointPtr joint = model->GetJoint(joint_name);     
        if (joint)
        {
          this->joints_.push_back(joint);
        }
        else
        {
          this->joints_.push_back(gazebo::physics::JointPtr());  // FIXME: cannot be null, must be an empty boost shared pointer
          RTT::log(RTT::Error) << "A joint named " << joint_name.c_str() << " is not part of Mechanism Controlled joints." << RTT::endlog();
        }
        
        //if(_sdf->HasElement(joint_name)) {
        //  double init = 0.0; //_sdf->GetElement(joint_name)->GetValueDouble();
        //  joint->SetAngle(0, init);
        //}
        
        stiffness_(i) = 200.0;
        damping_(i) = 5.0;
        trq_cmd_(i) = 0;
        joint_pos_cmd_(i) = joints_[i]->GetAngle(0).Radian();
      }
*/
      return true;
    }

void GetRobotChain()
{
/*  KDL::Tree my_tree;
  std::string robot_desc_string;
  rosnode_->param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }

  my_tree.getChain(chain_start, chain_end, chain_);
  
  dyn = new KDL::ChainDynParam(chain_, KDL::Vector(0.0, 0.0, -9.81));
  fk = new KDL::ChainFkSolverPos_recursive(chain_);
  jc = new KDL::ChainJntToJacSolver(chain_);
*/
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!model_dart_) {
        return;
    }

    const Eigen::MatrixXd &mass_matrix = dart_sk_->getMassMatrix();

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            r_MassMatrix_out_(i,j) = mass_matrix(r_indices_[i], r_indices_[j]);
            l_MassMatrix_out_(i,j) = mass_matrix(l_indices_[i], l_indices_[j]);
        }
    }

    port_r_MassMatrix_out_.write(r_MassMatrix_out_);
    port_l_MassMatrix_out_.write(l_MassMatrix_out_);


/*
  KDL::Frame f;
  KDL::Jacobian jac(7);
  KDL::JntSpaceInertiaMatrix H(7);
  KDL::JntArray pos(7);
  KDL::JntArray grav(7);
  Matrix77d mass;
  
  for(unsigned int i = 0; i< 7; i++)
  {
    jnt_pos_[i] = pos(i) = joint_pos_(i) = joints_[i]->GetAngle(0).Radian();
    jnt_vel_[i] =joint_vel_(i) = joints_[i]->GetVelocity(0);
  }

  dyn->JntToGravity(pos, grav);
  fk->JntToCart(pos, f);

  jc->JntToJac(pos, jac);
  jac.changeRefFrame(KDL::Frame(f.Inverse().M));
  dyn->JntToMass(pos, H);
  for(unsigned int i=0;i<7;i++) {
    for(unsigned int j=0;j<7;j++) {
      mass(i, j) = H.data(i, j);
    }
  }
  
  if (port_JointTorqueCommand.read(jnt_trq_cmd_) == RTT::NewData) {
    for (unsigned int i = 0; i < 7; i++) {
      trq_cmd_(i) = jnt_trq_cmd_(i);
    }
  }
  
  if (port_JointPositionCommand.read(jnt_pos_cmd_) == RTT::NewData) {
      for (unsigned int i = 0; i < 7; i++) {
        joint_pos_cmd_(i) = jnt_pos_cmd_(i);
      }
  }
  
  trq_ = stiffness_.asDiagonal() * (joint_pos_cmd_ - joint_pos_) - damping_.asDiagonal() * joint_vel_ + trq_cmd_;

  for(unsigned int i = 0; i< 7; i++) {
    joints_[i]->SetForce(0, trq_(i));
  }
  
  port_JointPosition.write(jnt_pos_);
  port_JointVelocity.write(jnt_vel_);
  port_JointTorque.write(jnt_trq_);

//  port_CartesianPosition.write(cart_pos);
//  port_CartesianVelocity.write(cart_twist);
//  port_CartesianWrench.write(cart_wrench);

  port_Jacobian.write(jac);
  port_MassMatrix.write(mass);
  */
}

  private:
    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    ros::NodeHandle* rosnode_;
    
    std::vector<gazebo::physics::JointPtr>  joints_;
    std::string chain_start, chain_end;
    
    
    KDL::Chain chain_;
    KDL::ChainDynParam *dyn;
    KDL::ChainFkSolverPos_recursive *fk;
    KDL::ChainJntToJacSolver *jc;

    std::string base_frame_;

    Eigen::Matrix<double, 7, 1> joint_pos_;
    Eigen::Matrix<double, 7, 1> joint_pos_cmd_;
    Eigen::Matrix<double, 7, 1> joint_vel_;
    Eigen::Matrix<double, 7, 1> stiffness_;
    Eigen::Matrix<double, 7, 1> damping_;
    Eigen::Matrix<double, 7, 1> trq_cmd_;
    Eigen::Matrix<double, 7, 1> trq_;
    
    std::vector<double> jnt_pos_;
    std::vector<double> jnt_trq_;
    std::vector<double> jnt_vel_;

    Eigen::VectorXd jnt_pos_cmd_;
    Eigen::VectorXd jnt_trq_cmd_;

    KDL::Frame T_old;
};



ORO_LIST_COMPONENT_TYPE(VelmaGazebo)
ORO_CREATE_COMPONENT_LIBRARY();

