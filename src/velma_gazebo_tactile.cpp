#include "velma_gazebo_tactile.h"
#include "barrett_hand_common/tactile_geometry.h"
#include "rtt_rosclock/rtt_rosclock.h"

    VelmaGazeboTactile::VelmaGazeboTactile(std::string const& name) : 
        TaskContext(name),
        median_filter_samples_(1),
        median_filter_max_samples_(8)
    {
        nh_ = new ros::NodeHandle();
        std::cout << "VelmaGazeboTactile ROS node namespace: " << nh_->getNamespace() << std::endl;

        std::cout << "VelmaGazeboTactile ROS node name: " << ros::this_node::getName() << std::endl;
        std::cout << "VelmaGazeboTactile ROS node namespace2: " << ros::this_node::getNamespace() << std::endl;

        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&VelmaGazeboTactile::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&VelmaGazeboTactile::gazeboUpdateHook,this,RTT::ClientThread);

        ts_[0] = new Tactile(median_filter_max_samples_);
        ts_[1] = new Tactile(median_filter_max_samples_);
        ts_[2] = new Tactile(median_filter_max_samples_);
        ts_[3] = new Tactile(median_filter_max_samples_);
        ts_[0]->setGeometry("finger1_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[1]->setGeometry("finger2_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[2]->setGeometry("finger3_tip_info", finger_sensor_center, finger_sensor_halfside1, finger_sensor_halfside2, 0.001);
        ts_[3]->setGeometry("palm_info", palm_sensor_center, palm_sensor_halfside1, palm_sensor_halfside2, 0.001);

        tactile_out_.finger1_tip.resize(24);
        tactile_out_.finger2_tip.resize(24);
        tactile_out_.finger3_tip.resize(24);
        tactile_out_.palm_tip.resize(24);

        this->ports()->addPort("BHPressureState_OUTPORT", port_tactile_out_);
        this->ports()->addPort("calibrate_tactile_sensors_INPORT", port_calibrate_in_);
        this->ports()->addPort("set_median_filter_INPORT", port_filter_in_);
        this->ports()->addPort("tactile_info_OUTPORT", port_tactile_info_out_);
        this->ports()->addPort("max_measured_pressure_OUTPORT", port_max_pressure_out_);
        max_pressure_out_.setZero();

		this->addProperty("prefix", prefix_);

        // tactile array info
        pressure_info_.sensor.resize(4);
        for (int id=0; id<4; ++id) {
            pressure_info_.sensor[id].frame_id = ts_[id]->getName();
            pressure_info_.sensor[id].center.resize(24);
            pressure_info_.sensor[id].halfside1.resize(24);
            pressure_info_.sensor[id].halfside2.resize(24);
            pressure_info_.sensor[id].force_per_unit.resize(24);
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

    VelmaGazeboTactile::~VelmaGazeboTactile() {
    }

    bool VelmaGazeboTactile::configureHook() {
        if (prefix_.empty()) {
            std::cout << "ERROR: VelmaGazeboTactile::configureHook: prefix is empty" << std::endl;
            return false;
        }

        std::string link_names[4] = {"_HandFingerOneKnuckleThreeLink", "_HandFingerTwoKnuckleThreeLink", "_HandFingerThreeKnuckleThreeLink", "_HandPalmLink"};
        for (int i = 0; i < 4; i++) {
            link_names_.push_back(prefix_ + link_names[i]);
        }

        std::cout << "VelmaGazeboTactile::configureHook: ok" << std::endl;
        return true;
    }

    void VelmaGazeboTactile::updateHook() {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);


            tactile_out_.header.stamp = rtt_rosclock::host_now();

            max_pressure_out_.setZero();
            for (int i=0; i<24; ++i)
            {
                tactile_out_.finger1_tip[i] = ts_[0]->getPressure(i,median_filter_samples_);
                tactile_out_.finger2_tip[i] = ts_[1]->getPressure(i,median_filter_samples_);
                tactile_out_.finger3_tip[i] = ts_[2]->getPressure(i,median_filter_samples_);
                tactile_out_.palm_tip[i] = ts_[3]->getPressure(i,median_filter_samples_);
                for (int puck_id = 0; puck_id < 4; puck_id++) {
                    if (max_pressure_out_(puck_id) < ts_[puck_id]->getPressure(i,median_filter_samples_)) {
                        max_pressure_out_(puck_id) = ts_[puck_id]->getPressure(i,median_filter_samples_);
                    }
                }
            }

            port_max_pressure_out_.write(max_pressure_out_);
            port_tactile_out_.write(tactile_out_);
            port_tactile_info_out_.write(pressure_info_);
    }

    bool VelmaGazeboTactile::startHook() {
      return true;
    }

    bool VelmaGazeboTactile::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

        if(model.get() == NULL) {
            std::cout << "VelmaGazeboTactile::gazeboConfigureHook: the gazebo model is NULL" << std::endl;
            return false;
        }

        model_ = model;

        dart_world_ = boost::dynamic_pointer_cast < gazebo::physics::DARTPhysics > ( gazebo::physics::get_world()->GetPhysicsEngine() ) -> GetDARTWorld();

        model_dart_ = boost::dynamic_pointer_cast < gazebo::physics::DARTModel >(model);
        if (model_dart_.get() == NULL) {
            std::cout << "VelmaGazebo::gazeboConfigureHook: the gazebo model is not a DART model" << std::endl;
            return false;
        }

        dart_sk_ = model_dart_->GetDARTSkeleton();

        detector_ = dart_world_->getConstraintSolver()->getCollisionDetector();

        return true;
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void VelmaGazeboTactile::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (link_names_.size() != 4) {
        std::cout << "ERROR: VelmaGazeboTactile: link_names_.size() != 4" << std::endl;
        return;
    }

    if (!model_dart_) {
        std::cout << "ERROR: VelmaGazeboTactile: !model_dart_" << std::endl;
        return;
    }

    RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(!trylock.isSuccessful()) {
        return;
    }

    Tactile::TactileState tact[4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 24; j++) {
            tact[i][j] = 0;
        }
    }

    detector_->detectCollision(true, true);
    size_t collisionCount = detector_->getNumContacts();
    if (collisionCount > 0) {
        for(size_t i = 0; i < collisionCount; ++i)
        {
            const dart::collision::Contact& contact = detector_->getContact(i);
            const std::string &b1_name = contact.bodyNode1->getName();
            const std::string &b2_name = contact.bodyNode2->getName();
            double closest_dist = -1;
            double max_force = -1;
            for (int lidx = 0; lidx < 4; lidx++) {
                bool check = false;
                Eigen::Isometry3d T_W_B;
                if (b1_name == link_names_[lidx]) {
                    check = true;
                    T_W_B = contact.bodyNode1->getTransform();
                }
                if (b2_name == link_names_[lidx]) {
                    check = true;
                    T_W_B = contact.bodyNode2->getTransform();
                }
                const Eigen::Vector3d &P_W = contact.point;
                const Eigen::Vector3d &P_B = T_W_B.inverse() * P_W;

                if (check) {
                    for (int tidx = 0; tidx < 24; tidx++) {
                        const Eigen::Isometry3d &T_S_B = ts_[lidx]->getFrameInv(tidx);
                        const Eigen::Vector3d &P_S = T_S_B * P_B;
                        if (std::fabs(P_S.x()) < ts_[lidx]->getHalfsideLength1(tidx) && std::fabs(P_S.y()) < ts_[lidx]->getHalfsideLength2(tidx)) {
                            tact[lidx][tidx] += 256;
                        }
                    }
                }
            }
        }
    }

    ts_[0]->updatePressure(tact[0]);
    ts_[1]->updatePressure(tact[1]);
    ts_[2]->updatePressure(tact[2]);
    ts_[3]->updatePressure(tact[3]);

}

ORO_LIST_COMPONENT_TYPE(VelmaGazeboTactile)
ORO_CREATE_COMPONENT_LIBRARY();

