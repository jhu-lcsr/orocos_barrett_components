#ifndef __ORO_BARRETT_SIM_HAND_SIM_DEVICE_H
#define __ORO_BARRETT_SIM_HAND_SIM_DEVICE_H

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <urdf/model.h>

#include <kdl/velocityprofile_trap.hpp>

#include <oro_barrett_interface/hand_device.h>

#include <gazebo/physics/physics.hh>

namespace oro_barrett_sim {

  /** \brief Orocos/ROS interface for a simulated Barrett Hand
   *
   */
  class HandSimDevice : public oro_barrett_interface::HandDevice
  {
  public:
    virtual void initialize();
    virtual void idle();
    virtual void run();

    virtual void setCompliance(bool enable);

    virtual void setTorqueMode(unsigned int joint_index);
    virtual void setPositionMode(unsigned int joint_index);
    virtual void setVelocityMode(unsigned int joint_index);
    virtual void setTrapezoidalMode(unsigned int joint_index);
    virtual void setIdleMode(unsigned int joint_index);

    virtual void readSim(ros::Time time, RTT::Seconds period);
    virtual void writeSim(ros::Time time, RTT::Seconds period);

    virtual void readDevice(ros::Time time, RTT::Seconds period);
    virtual void writeDevice(ros::Time time, RTT::Seconds period);

    virtual void open();
    virtual void close();

    HandSimDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        std::vector<gazebo::physics::JointPtr> joints);

  protected:

    bool doneMoving(const unsigned pair_index);
    bool withinTorqueLimits(const unsigned joint_index);

    void sampleTrapPosVel(const int &finger, const ros::Time &time, double &pos_sample, double &vel_sample) const;
    void sampleRampPosVel(const int &finger, const ros::Time &time, double &pos_sample, double &vel_sample) const;

    double getTargetPos(const unsigned dof, const ros::Time time);
    void setVelocity(const unsigned dof, const double vel);
    void setTrap(const unsigned dof, const double pos);

    const double outerCouplingForce(
      const double inner_pos,
      const double inner_vel,
      const double outer_pos,
      const double outer_vel) const;

    std::vector<gazebo::physics::JointPtr> gazebo_joints;

    bool compliance_enabled;

    Eigen::VectorXd
      link_torque,
      fingertip_torque,
      breakaway_angle,
      joint_i_err,
      joint_torque,
      joint_torque_max,
      joint_torque_breakaway,
      joint_velocity_cmd_start_positions;

    std::vector<ros::Time>
      joint_velocity_cmd_start_times;

    std::vector<KDL::VelocityProfile_Trap> trap_generators;
    std::vector<ros::Time> trap_start_times;
    std::vector<bool> torque_switches;

    double
      inner_breakaway_torque,
      stop_torque,
      max_torque,
      finger_acceleration,
      p_gain,
      d_gain,
      i_gain,
      i_clamp,
      trap_vel,
      trap_accel,
      velocity_gain,
      spread_p_gain,
      spread_d_gain,
      inner_breakaway_gain,
      outer_recouple_velocity,
      outer_coupling_p_gain,
      outer_coupling_d_gain;
  };
}

#endif // ifnedf __ORO_BARRETT_SIM_HAND_DEVICE_H
