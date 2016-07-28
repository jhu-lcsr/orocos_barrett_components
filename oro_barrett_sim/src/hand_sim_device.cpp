
#include <oro_barrett_sim/hand_sim_device.h>
#include <oro_barrett_msgs/BHandCmd.h>

#include <rtt_rosclock/rtt_rosclock.h>

//! Transmission ratio between two finger joints
static const double FINGER_JOINT_RATIO = 45.0/140.0;

static const unsigned SPREAD_ID = 3;

static const double FINGER_POS_MAX = 140.0/180.0*M_PI;

//! Get joint IDs from finger ID
static const bool fingerToJointIDs(const unsigned finger_id, unsigned &inner_id, unsigned &outer_id)
{
  if(finger_id < 3) {

    inner_id = finger_id + 2;
    outer_id = finger_id + 5;
  } else if(finger_id == 3) {
    inner_id = 0;
    outer_id = 1;
  } else {
    return false;
  }

  return true;
}

//! Get the sign of a scalar
template <typename T>
static const int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

static const double clamp(const double lower, const double value, const double upper)
{
  return std::max(lower,std::min(value,upper));
}

namespace oro_barrett_sim {

  HandSimDevice::HandSimDevice(
      RTT::Service::shared_ptr parent_service,
      const urdf::Model &urdf_model,
      const std::string &urdf_prefix,
      std::vector<gazebo::physics::JointPtr> joints) :
    oro_barrett_interface::HandDevice(
        parent_service,
        urdf_model,
        urdf_prefix),
    gazebo_joints(joints),
    compliance_enabled(false),
    link_torque(4),
    fingertip_torque(4),
    breakaway_angle(4),
    joint_i_err(8),
    joint_torque(8),
    joint_torque_max(Eigen::VectorXd::Constant(8, 1.5)),
    joint_torque_breakaway(4),
    joint_velocity_cmd_start_positions(4),
    joint_velocity_cmd_start_times(4),
    trap_start_times(4),
    torque_switches(4,false),
    torque_switch_positions(4,0.0),
    inner_breakaway_torque(1.5),
    stop_torque(2.0),
    admittance_control(false),
    admittance_gain(0.1),
    p_gain(1.0),
    i_gain(0.1),
    i_clamp(0.5),
    d_gain(0.01),
    trap_vel(1.0),
    trap_accel(10.0),
    velocity_gain(0.1),
    spread_p_gain(10.0),
    spread_d_gain(0.01),
    inner_breakaway_gain(10),
    outer_recouple_velocity(1.0)
  {
    // Initialize velocity profiles
    for(unsigned i=0; i<N_PUCKS; i++) {
      trap_generators.push_back(KDL::VelocityProfile_Trap(trap_vel,trap_accel));
    }

    //using namespace gazebo::physics;

    //ODELink *prox_link_1 = joints[2]->GetParent();
    //ODELink *prox_link_2 = joints[3]->GetParent();

    RTT::Service::shared_ptr hand_service = parent_service->provides("hand");

    hand_service->addProperty("trap_vel",trap_vel);
    hand_service->addProperty("trap_accel",trap_accel);

    hand_service->addProperty("max_torque",max_torque);
    hand_service->addProperty("stop_torque",stop_torque);
    hand_service->addProperty("inner_breakaway_torque",inner_breakaway_torque);
    hand_service->addProperty("inner_breakaway_gain",inner_breakaway_gain);
    hand_service->addProperty("finger_acceleration",finger_acceleration);
    hand_service->addProperty("velocity_gain",velocity_gain);
    hand_service->addProperty("outer_recouple_velocity",outer_recouple_velocity);

    hand_service->addProperty("admittance_control",admittance_control);
    hand_service->addProperty("admittance_gain",admittance_gain);

    hand_service->addProperty("p_gain",p_gain);
    hand_service->addProperty("i_gain",i_gain);
    hand_service->addProperty("i_clamp",i_clamp);
    hand_service->addProperty("d_gain",d_gain);

    hand_service->addProperty("spread_d_gain",spread_d_gain);
    hand_service->addProperty("spread_p_gain",spread_p_gain);

    hand_service->addProperty("outer_coupling_p_gain",outer_coupling_p_gain);
    hand_service->addProperty("outer_coupling_d_gain",outer_coupling_d_gain);

    joint_torque_cmd.resize(N_PUCKS);
    joint_position_cmd.resize(N_PUCKS);
    joint_velocity_cmd.resize(N_PUCKS);
  }

  void HandSimDevice::initialize()
  {
    init_state = INIT_FINGERS;
    run_mode = INITIALIZE;

    joint_position.setZero();
    joint_velocity.setZero();

    link_torque.setZero();
    fingertip_torque.setZero();
    joint_i_err.setZero();

    joint_cmd.cmd.assign(0);

    // Set trajectory generator parameters
    for(unsigned i=0; i<N_PUCKS; i++) {
      trap_generators[i].SetMax(trap_vel, trap_accel);
    }
  }

  void HandSimDevice::idle()
  {
    run_mode = IDLE;
    joint_cmd.mode.assign(oro_barrett_msgs::BHandCmd::MODE_IDLE);
  }

  void HandSimDevice::run()
  {
    run_mode = RUN;
    joint_cmd.mode.assign(oro_barrett_msgs::BHandCmd::MODE_PID);
    if(admittance_control) {
      for(int i=0; i<N_PUCKS; i++) {
        if(i!=SPREAD_ID) {
          unsigned inner, outer;
          fingerToJointIDs(i, inner, outer);
          gazebo_joints[inner]->SetMaxForce(0, 100);
          gazebo_joints[outer]->SetMaxForce(0, 100);
        }
      }
    }
  }

  void HandSimDevice::setCompliance(bool enable)
  {
    compliance_enabled = enable;
  }

  bool HandSimDevice::withinTorqueLimits(const unsigned joint_id) {
    return std::abs(joint_torque[joint_id]) > joint_torque_max[joint_id];
  }

  void HandSimDevice::sampleTrapPosVel(const int &dof, const ros::Time &time, double &pos_sample, double &vel_sample) const
  {
    const RTT::Seconds sample_secs = (time - trap_start_times[dof]).toSec();
    pos_sample = trap_generators[dof].Pos(sample_secs);
    vel_sample = trap_generators[dof].Vel(sample_secs);
  }

  void HandSimDevice::sampleRampPosVel(const int &dof, const ros::Time &time, double &pos_sample, double &vel_sample) const
  {
    vel_sample = joint_cmd.cmd[dof];
    pos_sample =
      joint_velocity_cmd_start_positions[dof] +
      vel_sample * (time - joint_velocity_cmd_start_times[dof]).toSec();
    pos_sample = clamp(0, pos_sample, (dof == 3) ? M_PI : FINGER_POS_MAX);
  }

  const double HandSimDevice::outerCouplingForce(
      const double inner_pos,
      const double inner_vel,
      const double outer_pos,
      const double outer_vel) const
  {
    return
      outer_coupling_p_gain * (FINGER_JOINT_RATIO*inner_pos - outer_pos)
      + outer_coupling_d_gain * (FINGER_JOINT_RATIO*inner_vel - outer_vel);
  }

  const double HandSimDevice::innerBreakawayForce(
      const double inner_pos,
      const double inner_vel,
      const double torque_switch_position) const
  {
    return
      outer_coupling_p_gain * (torque_switch_position - inner_pos)
      + outer_coupling_d_gain * (0.0 - inner_vel);
  }


  void HandSimDevice::readSim(ros::Time time, RTT::Seconds period)
  {
    // torque filter constant
    const double t = exp(-2.0 * M_PI * period / 0.02);

    // Get state from ALL gazebo joints
    for(unsigned i=0; i < N_PUCKS; i++) {
      // Get joint torques
      unsigned inner, outer;
      fingerToJointIDs(i, inner, outer);

      double inner_pos = (1.0-t)*joint_position[inner] + t*gazebo_joints[inner]->GetAngle(0).Radian();
      double outer_pos = (1.0-t)*joint_position[outer] + t*gazebo_joints[outer]->GetAngle(0).Radian();

      double inner_vel = (inner_pos - joint_position[inner])/period;
      double outer_vel = (outer_pos - joint_position[outer])/period;
      if(std::abs(inner_vel) > 10 || period < 1E-6) { inner_vel = 0; }
      if(std::abs(outer_vel) > 10 || period < 1E-6) { outer_vel = 0; }

      joint_position[inner] = inner_pos;
      joint_position[outer] = outer_pos;

      joint_velocity[inner] = inner_vel;
      joint_velocity[outer] = outer_vel;

      if(i == SPREAD_ID) {
        joint_torque[inner] = gazebo_joints[inner]->GetForce(0u);
        joint_torque[outer] = gazebo_joints[outer]->GetForce(0u);
      } else {
        link_torque[i] =  (1.0-t)*link_torque[i] + t*gazebo_joints[inner]->GetForceTorque(0).body2Torque.z;
        fingertip_torque[i] = (1.0-t)*fingertip_torque[i] + t*gazebo_joints[outer]->GetForceTorque(0).body2Torque.z;

        joint_torque[inner] = gazebo_joints[inner]->GetForce(0u);
        joint_torque[outer] = gazebo_joints[outer]->GetForce(0u);
      }
    }
  }

  void HandSimDevice::writeSim(ros::Time time, RTT::Seconds period)
  {
    for(unsigned i=0; i<4; i++) {

      const double &cmd = joint_cmd.cmd[i];

      if(i == SPREAD_ID)
      {
        // Set the finger indices (for convenience)
        const unsigned f1=0, f2=1;

        // Spread: both proximal joints should be kept at the same position
        const double spread_err = joint_position[f1] - joint_position[f2];
        const double spread_derr = joint_velocity[f1] - joint_velocity[f2];
        const double spread_constraint_force = spread_p_gain*spread_err + spread_d_gain*spread_derr;

        // F1 and F2 spread efforts
        double eff_cmd_f1 = 0;
        double eff_cmd_f2 = 0;

        if(joint_cmd.mode[SPREAD_ID] == oro_barrett_msgs::BHandCmd::MODE_TORQUE)
        {
          // Set effort directly
          eff_cmd_f1 = cmd;
          eff_cmd_f2 = -cmd;
        }
        else
        {
          // Get the PID refs
          double pos_cmd = 0;
          double vel_cmd = 0;

          switch(joint_cmd.mode[SPREAD_ID])
          {
            case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
              this->sampleTrapPosVel(SPREAD_ID, time, pos_cmd, vel_cmd);
              break;
            case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
              this->sampleRampPosVel(SPREAD_ID, time, pos_cmd, vel_cmd);
              break;
            case oro_barrett_msgs::BHandCmd::MODE_PID:
              pos_cmd = joint_cmd.cmd[SPREAD_ID];
              vel_cmd = 0.0;
              break;
            default:
              RTT::log(RTT::Error) << "Bad high-level finger command mode: "<<int(joint_cmd.mode[SPREAD_ID])<<RTT::endlog();
              return;
          };

          // Clip position command
          pos_cmd = clamp(0.0, pos_cmd, M_PI);

          // Compute PID command
          const double p_err_f1 = pos_cmd - joint_position[f1];
          const double p_err_f2 = pos_cmd - joint_position[f2];

          const double d_err_f1 = vel_cmd - joint_velocity[f1];
          const double d_err_f2 = vel_cmd - joint_velocity[f2];

          joint_i_err[f1] = clamp(-i_clamp, joint_i_err[f1] + p_err_f1*period, i_clamp);
          joint_i_err[f2] = clamp(-i_clamp, joint_i_err[f2] + p_err_f1*period, i_clamp);

          eff_cmd_f1 = p_gain*p_err_f1 + i_gain*joint_i_err[f1] + d_gain*d_err_f1;
          eff_cmd_f2 = p_gain*p_err_f2 + i_gain*joint_i_err[f2] + d_gain*d_err_f2;
        }

        // Set the spread effort
        gazebo_joints[f1]->SetForce(0, eff_cmd_f1 - spread_constraint_force);
        gazebo_joints[f2]->SetForce(0, eff_cmd_f2 + spread_constraint_force);
      }
      else
      {
        // TorqueSwitch simulation
        //
        // The following code aims to reproduce a behavior similar to the
        // TorqueSwitch mechanism of the real Barrett Hand.
        //
        // Before the inner link's motion is obstructed, the outer link's
        // position is linearly coupled to it by the finger joint ratio.
        //
        // When the inner link encounters a torque larger than the "breakaway"
        // torque, the TorqueSwitch engages and the outer link begins to move
        // inward independently.

        // Get the finger indices (for convenience)
        unsigned inner, outer;
        fingerToJointIDs(i, inner, outer);

        const double &inner_pos = joint_position[inner];
        const double &outer_pos = joint_position[outer];

        const double &inner_vel = joint_velocity[inner];
        const double &outer_vel = joint_velocity[outer];

        // Fingers: handle TorqueSwitch semi-underactuated behavior
        const bool coupled = !torque_switches[i];

        // Final effort command
        double eff_cmd = 0.0;

        // Switch the control law based on the command mode
        if(joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_TORQUE)
        {
          eff_cmd = cmd;
        }
        else
        {
          // Get PID refs
          double pos_cmd = 0;
          double vel_cmd = 0;

          switch(joint_cmd.mode[i])
          {
            case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
              this->sampleTrapPosVel(i, time, pos_cmd, vel_cmd);
              break;
            case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
              this->sampleRampPosVel(i, time, pos_cmd, vel_cmd);
              break;
            case oro_barrett_msgs::BHandCmd::MODE_PID:
              pos_cmd = cmd;
              vel_cmd = 0.0;
              break;
            default:
              RTT::log(RTT::Error) << "Bad high-level command mode: "<<int(joint_cmd.mode[i])<<RTT::endlog();
              return;
          };

          // Don't try to command past the joint limits
          pos_cmd = clamp(0.0, pos_cmd, FINGER_POS_MAX);

          // Compute PID commands
          if(coupled) {
            // Update integral error
            const double p_err = pos_cmd - inner_pos;
            joint_i_err[inner] = clamp(-i_clamp, joint_i_err[inner] + p_err*period, i_clamp);

            // Command applied to inner joint
            eff_cmd = p_gain * (p_err) + i_gain * joint_i_err[inner] + d_gain * (vel_cmd - inner_vel);
          } else {
            // Command applied to outer joint
            eff_cmd = outerCouplingForce(pos_cmd, vel_cmd, outer_pos, outer_vel);
          }
        }

        // Apply inner and outer effort
        double inner_torque = 0.0;
        double outer_torque = 0.0;
        if(coupled) {
          inner_torque = eff_cmd;
          outer_torque = outerCouplingForce(inner_pos, inner_vel, outer_pos, outer_vel);
        } else {
          inner_torque = innerBreakawayForce(inner_pos, inner_vel, torque_switch_positions[i]);
          outer_torque = eff_cmd;
        }

        if(not admittance_control) {
          // Impedance-based
          gazebo_joints[inner]->SetForce(0, inner_torque);
          gazebo_joints[outer]->SetForce(0, outer_torque);
        } else {
          // Admittance-based
          gazebo_joints[inner]->SetVelocity(0, admittance_gain*(inner_torque - link_torque[i]));
          gazebo_joints[outer]->SetVelocity(0, admittance_gain*(outer_torque - fingertip_torque[i]));
        }

        // Update the torque switch state
        if(!torque_switches[i]) {
          // Check for torque switch engage condition
          if(link_torque[i] > inner_breakaway_torque and inner_vel < 0.01) {
            RTT::log(RTT::Warning) << "Enabling torque switch for F" << i+1 << RTT::endlog();
            torque_switches[i] = true;
            torque_switch_positions[i] = inner_pos;
          }
        } else {
          // Check for torque switch disengage condition
          if(eff_cmd < 0 and (outer_pos <= inner_pos/FINGER_JOINT_RATIO)) {
            RTT::log(RTT::Warning) << "Disabling torque switch for F" << i+1 << RTT::endlog();
            torque_switches[i] = false;
          }
        }

      }
    }
  }


  void HandSimDevice::readDevice(ros::Time time, RTT::Seconds period)
  {
    // Always compute and write center of mass
    this->computeCenterOfMass(center_of_mass);
    center_of_mass_out.write(center_of_mass);

    // Get the state, and re-shape it
    joint_position_out.write(joint_position);

    // Publish state to ROS
    if(this->joint_state_throttle.ready(0.01)) {
      // Update the joint state message
      this->joint_state.header.stamp = rtt_rosclock::host_now();
      this->joint_state.name = this->joint_names;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),8) = this->joint_position;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),8) = this->joint_velocity;
      Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),8) = this->joint_torque;

      // Publish
      this->joint_state_out.write(this->joint_state);

      // Create a pose structure from the center of mass
      com_msg.header.stamp = rtt_rosclock::host_now();
      com_msg.pose.position.x = center_of_mass[0];
      com_msg.pose.position.y = center_of_mass[1];
      com_msg.pose.position.z = center_of_mass[2];
      this->center_of_mass_debug_out.write(com_msg);

      // Write out hand status
      this->status_msg.header.stamp = this->joint_state.header.stamp;
      this->status_msg.temperature.assign(25.0);
      for(unsigned i=0; i<4; i++) {
        this->status_msg.mode[i] = this->joint_cmd.mode[i];
      }
      this->status_out.write(this->status_msg);
    }
  }

  void HandSimDevice::writeDevice(ros::Time time, RTT::Seconds period)
  {
    switch(run_mode) {
      case IDLE:
        // Don't command the hand
        break;
      case INITIALIZE:
        {
          switch(init_state) {
            case INIT_FINGERS:
              init_state = SEEK_FINGERS;
              open();
              break;
            case SEEK_FINGERS:
              if(doneMoving(0) && doneMoving(1) && doneMoving(2)) {
                init_state = SEEK_SPREAD;
              }
              break;
            case SEEK_SPREAD:
              if(doneMoving(3)) {
                init_state = INIT_CLOSE;
              }
              break;
            case INIT_CLOSE:
              close();
              run_mode = RUN;
              break;
          };
          break;
        }
      case RUN:
        {
          // Read commands
          bool new_torque_cmd = (joint_torque_in.readNewest(joint_torque_cmd) == RTT::NewData);
          bool new_position_cmd = (joint_position_in.readNewest(joint_position_cmd) == RTT::NewData);
          bool new_velocity_cmd = (joint_velocity_in.readNewest(joint_velocity_cmd) == RTT::NewData);
          bool new_trapezoidal_cmd = (joint_trapezoidal_in.readNewest(joint_trapezoidal_cmd) == RTT::NewData);

          // Check low-level command sizes
          if(joint_torque_cmd.size() != N_PUCKS ||
             joint_position_cmd.size() != N_PUCKS ||
             joint_velocity_cmd.size() != N_PUCKS ||
             joint_trapezoidal_cmd.size() != N_PUCKS)
          {
            RTT::log(RTT::Error) << "Input command size mismatch!" << RTT::endlog();
            idle();
            return;
          }

          // Get full ROS message command (overrides low-level commands)
          oro_barrett_msgs::BHandCmd joint_cmd_tmp;
          bool new_joint_cmd = (joint_cmd_in.readNewest(joint_cmd_tmp) == RTT::NewData);

          for(int i=0; i<N_PUCKS; i++) {
            // Initialize the new command mode to remain the same
            int new_cmd_mode = joint_cmd.mode[i];

            // Update command vectors with input from ROS message
            if(new_joint_cmd)
            {
              switch(joint_cmd_tmp.mode[i])
              {
                case oro_barrett_msgs::BHandCmd::MODE_SAME:
                  // Ignore this puck, don't update command or command mode
                  joint_cmd_tmp.mode[i] = joint_cmd.mode[i];
                  continue;
                case oro_barrett_msgs::BHandCmd::MODE_IDLE:
                  joint_velocity_cmd[i] = 0;
                  new_velocity_cmd = true;
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_TORQUE:
                  joint_torque_cmd[i] = joint_cmd_tmp.cmd[i];
                  new_torque_cmd = true;
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_PID:
                  joint_position_cmd[i] = joint_cmd_tmp.cmd[i];
                  new_position_cmd = true;
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
                  joint_velocity_cmd[i] = joint_cmd_tmp.cmd[i];
                  new_velocity_cmd = true;
                  break;
                case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
                  joint_trapezoidal_cmd[i] = joint_cmd_tmp.cmd[i];
                  new_trapezoidal_cmd = true;
                  break;
                default:
                  RTT::log(RTT::Error) << "Bad BHand command mode: "<< (int)joint_cmd_tmp.mode[i] << RTT::endlog();
                  return;
              };

              // Get the new command mode
              new_cmd_mode = int(joint_cmd_tmp.mode[i]);
            }

            // Update the command
            if(new_torque_cmd && new_cmd_mode == oro_barrett_msgs::BHandCmd::MODE_TORQUE) {
              // Set the new torqe command
              joint_cmd.mode[i] = new_cmd_mode;
              joint_cmd.cmd[i] = joint_torque_cmd[i];
            } else if(new_position_cmd && new_cmd_mode == oro_barrett_msgs::BHandCmd::MODE_PID) {
              // Set the new PID command
              joint_cmd.mode[i] = new_cmd_mode;
              joint_cmd.cmd[i] = joint_position_cmd[i];
            } else if(new_velocity_cmd && new_cmd_mode == oro_barrett_msgs::BHandCmd::MODE_VELOCITY) {
              // Initialize a position ramp
              this->setVelocity(i, joint_velocity_cmd[i]);
            } else if(new_trapezoidal_cmd && new_cmd_mode == oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL) {
              // Initialize a trapezoidal generator
              this->setTrap(i, joint_trapezoidal_cmd[i]);
            }
          }

        }
        break;
    };
  }


  void HandSimDevice::setTorqueMode(unsigned int joint_index)
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TORQUE;
  }
  void HandSimDevice::setPositionMode(unsigned int joint_index)
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_PID;
  }
  void HandSimDevice::setVelocityMode(unsigned int joint_index)
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
  }
  void HandSimDevice::setTrapezoidalMode(unsigned int joint_index)
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;
  }
  void HandSimDevice::setIdleMode(unsigned int joint_index)
  {
    joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_IDLE;
  }

  bool HandSimDevice::doneMoving(const unsigned pair_index)
  {
    unsigned inner_id, outer_id;
    fingerToJointIDs(pair_index, inner_id, outer_id);

    return joint_velocity[inner_id] < 0.01 && joint_velocity[outer_id] < 0.01;
  }

  void HandSimDevice::open()
  {
    for(int i=0; i<3; i++) {
      this->setVelocity(i, -1.0);
    }
  }

  void HandSimDevice::close()
  {
    for(int i=0; i<3; i++) {
      this->setVelocity(i, 1.0);
    }
  }

  double HandSimDevice::getTargetPos(const unsigned dof, const ros::Time time)
  {
    unsigned inner_id = 0, outer_id = 0;
    fingerToJointIDs(dof, inner_id, outer_id);

    double target_pos;
    double target_vel;

    switch(joint_cmd.mode[dof])
    {
      case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
        this->sampleTrapPosVel(dof, time, target_pos, target_vel);
        break;
      case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
        this->sampleRampPosVel(dof, time, target_pos, target_vel);
        break;
      case oro_barrett_msgs::BHandCmd::MODE_PID:
        target_pos = joint_cmd.cmd[dof];
        break;
      default:
        target_pos = joint_position[inner_id];
        break;
    };

    return target_pos;
  }

  void HandSimDevice::setVelocity(const unsigned dof, const double vel)
  {
    ros::Time time = rtt_rosclock::rtt_now();

    const double start_pos = getTargetPos(dof, time);

    joint_velocity_cmd_start_times[dof] = time;
    joint_velocity_cmd_start_positions[dof] = start_pos;

    joint_cmd.mode[dof] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
    joint_cmd.cmd[dof] = vel;
  }

  void HandSimDevice::setTrap(const unsigned dof, const double pos)
  {
    ros::Time time = rtt_rosclock::rtt_now();

#if 0 // this isn't how the real hand works
    // Check if the current trap is suitable with a new second position
    bool pursuing_trap = joint_cmd.mode[dof] == oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;

    if(pursuing_trap) {
      KDL::VelocityProfile_Trap &current_trap = trap_generators[i];

      double current_target = joint_cmd.cmd[dof];
      double target_diff = pos - current_target;

      if(sgn(target_diff) == sgn(current_trap.Vel((time-trap_start_times[i])))) {
        // Extend the target position of the current trap
        trap_generators[dof].SetProfile(current_trap.Pos(0.0), pos);
      }
    }
#endif

    const double start_pos = getTargetPos(dof, time);

    trap_start_times[dof] = time;
    trap_generators[dof].SetProfile(start_pos, pos);

    joint_cmd.mode[dof] = oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;
    joint_cmd.cmd[dof] = pos;
  }
}
