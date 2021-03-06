
#include <rtt/Component.hpp>

#include <oro_barrett_sim/barrett_sim_manager.h>

#include <boost/assign/list_of.hpp>

#include <rtt_rosparam/rosparam.h>

using namespace oro_barrett_sim;

BarrettSimManager::BarrettSimManager(const std::string &name) :
  oro_barrett_interface::BarrettManager(name),
  bus_id_(0),
  config_path_(""),
  last_gz_update_time_(0,0)
{
  this->addProperty("bus_id",bus_id_)
    .doc("The CANBus port [0-n]. (UNUSED)");

  this->addProperty("config_path",config_path_)
    .doc("The path to the libbarrett config file (leave blank for default path). (UNUSED)");

  this->addProperty("real_period",period_);
  this->addProperty("read_duration",read_duration_);
  this->addProperty("write_duration",write_duration_);
  this->addProperty("gz_duration",gz_duration_);
  this->addProperty("gz_wam_duration",gz_wam_duration_);
  this->addProperty("gz_hand_duration",gz_hand_duration_);

  // Add required gazebo interfaces
  this->provides("gazebo")->addProperty("last_gz_update_time",last_gz_update_time_);
  this->provides("gazebo")->addOperation("configure",&BarrettSimManager::gazeboConfigureHook,this,RTT::ClientThread);
  this->provides("gazebo")->addOperation("update",&BarrettSimManager::gazeboUpdateHook,this,RTT::ClientThread);
}

//! Called from gazebo
bool BarrettSimManager::gazeboConfigureHook(gazebo::physics::ModelPtr model)
{
  RTT::log(RTT::Info) << "Configuring Barrett manager from Gazebo Model..." << RTT::endlog();

  if(model.get() == NULL) {
    RTT::log(RTT::Error) << "Barrett Gazebo Model was null." << RTT::endlog();
    return false;
  }

  // Save the model
  gazebo_model_ = model;

  return true;
}

//! Called from Gazebo
void BarrettSimManager::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
  ros::Time gz_time = rtt_rosclock::rtt_now();
  RTT::Seconds gz_period = (gz_time - last_gz_update_time_).toSec();
  if(gz_period < 1.0E-5) {
      RTT::log(RTT::Debug) << "BarrettSimManager::gazeboUpdateHook period:" <<gz_period << RTT::endlog();
      return;
  }
  gz_period_ = gz_period;

  if(!model) {
    RTT::log(RTT::Error) << "BarrettSimManager::gazeboUpdateHook called with null model." << RTT::endlog();
    return;
  }

  //RTT::log(RTT::Debug) << ">>> BarrettSimManager::gazeboUpdateHook begin" << RTT::endlog();

  ros::Time wall_time = rtt_rosclock::host_wall_now();
  if(wam_device_) {
    wam_device_->readSim(gz_time, gz_period_);
    wam_device_->writeSim(gz_time, gz_period_);
  }
  gz_wam_duration_ = (rtt_rosclock::host_wall_now() - wall_time).toSec();

  wall_time = rtt_rosclock::host_wall_now();
  if(hand_device_) {
    hand_device_->readSim(gz_time, gz_period_);
    hand_device_->writeSim(gz_time, gz_period_);
  }
  gz_hand_duration_ = (rtt_rosclock::host_wall_now() - wall_time).toSec();

  gz_duration_ = gz_wam_duration_ + gz_hand_duration_;

  last_gz_update_time_ = gz_time;
  //RTT::log(RTT::Debug) << "<<< BarrettSimManager::gazeboUpdateHook end" << RTT::endlog();
}

bool BarrettSimManager::configureHook()
{
  RTT::Logger::Instance()->in("BarrettSimManager::configureHook");

  // Get the rosparam service requester
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");

  if(!rosparam) {
    RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
    return false;
  }

  // Get the ROS parameters
  rosparam->getAllComponentPrivate();

  // Make sure the gazebo hook has been configured
  if(!gazebo_model_) {
    RTT::log(RTT::Error) << "No gazebo model defined." << RTT::endlog();
    return false;
  }

  // Parse URDF from string
  if(!urdf_model_.initString(urdf_str_)) {
    return false;
  }

  // Auto-configure optional WAM
  if(auto_configure_wam_) {
    RTT::log(RTT::Info) << "Configuring WAM..." << RTT::endlog();
    switch(wam_dof_) {
      case 4:
        if(!this->configureWam4(wam_urdf_prefix_)) {
          RTT::log(RTT::Error) << "Unable to auto-configure 4-DOF WAM with URDF prefix \""<<wam_urdf_prefix_<<"\"." <<RTT::endlog();
          return false;
        }
        break;
      case 7:
        if(!this->configureWam7(wam_urdf_prefix_)) {
          RTT::log(RTT::Error) << "Unable to auto-configure 7-DOF WAM with URDF prefix \""<<wam_urdf_prefix_<<"\"." <<RTT::endlog();
          return false;
        }
        break;
      default:
        RTT::log(RTT::Error) << "Unable to auto-configure WAM with URDF prefix "
          "\""<<wam_urdf_prefix_<<"\". DOF should be 4 or 7, but it's "
          <<wam_dof_<<"." <<RTT::endlog();
        return false;
    };

    // Get WAM ros parameters
    rosparam->getComponentPrivate("wam");
  }

  // Auto-configure optional BHand
  if(auto_configure_hand_) {
    RTT::log(RTT::Info) << "Configuring hand with URDF prefix \""<<hand_urdf_prefix_<<"\"..." << RTT::endlog();
    if(!this->configureHand(hand_urdf_prefix_)) {
      RTT::log(RTT::Error) << "Unable to auto-configure BHand with URDF prefix \""<<hand_urdf_prefix_<<"\"." <<RTT::endlog();
      return false;
    }

    // Get BHand ros parameters
    rosparam->getComponentPrivate("hand");
  }

  return true;
}


bool BarrettSimManager::startHook()
{

  // Initialize the last update time
  last_update_time_ = rtt_rosclock::rtt_now();

  // Read the state configuration
  try {
    if(wam_device_) {
      RTT::log(RTT::Debug) << "Initializing WAM..." << RTT::endlog();
      // Zero the commands before the arm is activated
      wam_device_->setZero();
      // Write to the configuration ports
      wam_device_->readConfig();
      wam_device_->readDevice(last_update_time_,this->getPeriod());
    }
  } catch(std::runtime_error &err) {
    RTT::log(RTT::Error) << "Could not start WAM: " << err.what() << RTT::endlog();
    return false;
  }

  return true;
}

void BarrettSimManager::updateHook()
{
  ros::Time time = rtt_rosclock::rtt_now();
  RTT::Seconds period = (time - last_update_time_).toSec();
  period_ = period;

  //RTT::log(RTT::Debug) << ">>> BarrettSimManager::updateHook begin" << RTT::endlog();

  // Read
  RTT::os::TimeService::ticks read_start = RTT::os::TimeService::Instance()->getTicks();
  if(wam_device_) {
    try {
      // Read the state estimation
      wam_device_->readDevice(time,period);
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not read the WAM state: " << err.what() << RTT::endlog();
      this->error();
    }
  }
  if(hand_device_) {
    try {
      // Read the state estimation
      hand_device_->readDevice(time,period);
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not read the BHand state: " << err.what() << RTT::endlog();
      this->error();
    }
  }
  read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

  // Write
  RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();
  if(wam_device_) {
    try {
      // Write the control command (force the write if the system is idle)
      wam_device_->writeDevice(time,period);
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not write the WAM command: " << err.what() << RTT::endlog();
      this->error();
    }
  }
  if(hand_device_) {
    try {
      // Write the control command (force the write if the system is idle)
      hand_device_->writeDevice(time,period);
    } catch(std::runtime_error &err) {
      RTT::log(RTT::Error) << "Could not write the BHand command: " << err.what() << RTT::endlog();
      this->error();
    }
  }
  write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);

  last_update_time_ = time;
  //RTT::log(RTT::Debug) << "<<< BarrettSimManager::updateHook end" << RTT::endlog();
}

void BarrettSimManager::stopHook()
{
  // Set the mode to IDLE
  if(hand_device_) {
    hand_device_->idle();
  }
}

void BarrettSimManager::cleanupHook()
{
  wam_device_.reset();
}

bool BarrettSimManager::configureWam4(const std::string &urdf_prefix)
{
  if(!this->configureWam<4>(urdf_prefix)) {
    this->provides()->removeService("wam");
    return false;
  }

  return true;
}
bool BarrettSimManager::configureWam7(const std::string &urdf_prefix)
{
  if(!this->configureWam<7>(urdf_prefix)) {
    this->provides()->removeService("wam");
    return false;
  }

  return true;
}

  template<size_t DOF>
bool BarrettSimManager::configureWam(const std::string &urdf_prefix)
{
  RTT::Logger::Instance()->in("BarrettSimManager::configureWam");
  using namespace oro_barrett_interface;

  // Make sure we 're in the configured state, and not running
  if(!this->isConfigured() || this->isRunning()) {
    RTT::log(RTT::Error) << "Cannot configure WAM while the component is "
      "unconfigured or running." << RTT::endlog();
    return false;
  }

  // Check for a 7-DOF WAM
  std::vector<std::string> wam_joint_names = boost::assign::list_of
    ("/base_yaw_joint")
    ("/shoulder_pitch_joint")
    ("/shoulder_yaw_joint")
    ("/elbow_pitch_joint")
    ("/wrist_yaw_joint")
    ("/wrist_pitch_joint")
    ("/palm_yaw_joint");

  // Vector of gazebo joints
  std::vector<gazebo::physics::JointPtr> joints(DOF);

  // Look at the SDF for the 7 desired WAM joints
  for(unsigned j=0; j<DOF; j++) {
    joints[j] = gazebo_model_->GetJoint(urdf_prefix + wam_joint_names[j]);
    if(!joints[j]) {
      RTT::log(RTT::Error) << "Could not find joint \"" << urdf_prefix+wam_joint_names[j] <<"\" in URDF/SDF"<<RTT::endlog();

      RTT::log(RTT::Error) << "Joints are:" << RTT::endlog();
      const std::vector<gazebo::physics::JointPtr> &joints = gazebo_model_->GetJoints();
      for(unsigned ja=0; ja<joints.size(); ja++) {
        RTT::log(RTT::Error) << " -- " << joints[ja]->GetName() << RTT::endlog();
      }

      return false;
    }
  }

  try{
    // Construct a new wam device and "wam" service (interface and state storage)
    if(!wam_device_) {
      RTT::log(RTT::Info) << "Configuring " <<DOF<<"-DOF WAM"<< RTT::endlog();
      wam_device_.reset(
          new WamSimDevice<DOF>(
            this->provides(),
            urdf_model_,
            urdf_prefix,
            joints));
    }
  } catch(std::runtime_error &ex) {
    RTT::log(RTT::Error) << "Could not configure " << DOF << "-DOF WAM" << ex.what() << RTT::endlog();
    wam_device_.reset();
    return false;
  }

  return true;
}


bool BarrettSimManager::configureHand(const std::string &urdf_prefix)
{
  RTT::Logger::Instance()->in("BarrettSimManager::configureHand");
  using namespace oro_barrett_interface;

  std::vector<std::string> hand_joint_names = boost::assign::list_of
    ("/finger_1/prox_joint")
    ("/finger_2/prox_joint")
    ("/finger_1/med_joint")
    ("/finger_2/med_joint")
    ("/finger_3/med_joint")
    ("/finger_1/dist_joint")
    ("/finger_2/dist_joint")
    ("/finger_3/dist_joint");

  // Vector of gazebo joints
  std::vector<gazebo::physics::JointPtr> joints(hand_joint_names.size());

  // Look at the SDF for the 7 desired WAM joints
  for(unsigned j=0; j<oro_barrett_interface::HandDevice::DOF; j++) {
    joints[j] = gazebo_model_->GetJoint(urdf_prefix + hand_joint_names[j]);
    if(!joints[j]) {
      RTT::log(RTT::Error) << "Could not find joint \"" << urdf_prefix+hand_joint_names[j] <<"\" in URDF/SDF"<<RTT::endlog();
      return false;
    }
  }

  try{
    // Construct a new bhand device and "hand" service (interface and state storage)
    hand_device_.reset();
    hand_device_.reset(
        new HandSimDevice(
          this->provides(),
          urdf_model_,
          urdf_prefix,
          joints));
  } catch(std::runtime_error &ex) {
    RTT::log(RTT::Error) << "Could not configure Barrett Hand: " <<
      ex.what() << RTT::endlog();
    return false;
  } catch(...) {
    RTT::log(RTT::Error) << "Could not configure Barrett Hand." << RTT::endlog();
    throw;
  }
  RTT::log(RTT::Info) << "Configured Barrett Hand." << RTT::endlog();

  return true;
}

ORO_LIST_COMPONENT_TYPE(oro_barrett_sim::BarrettSimManager);
ORO_CREATE_COMPONENT_LIBRARY()

