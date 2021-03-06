#ifndef __ORO_BARRETT_SIM_WAM_SIM_DEVICE
#define __ORO_BARRETT_SIM_WAM_SIM_DEVICE

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

#include <oro_barrett_interface/wam_device.h>

#include <sensor_msgs/JointState.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <angles/angles.h>

#include <gazebo/physics/physics.hh>

#include <oro_barrett_msgs/BarrettStatus.h>


namespace oro_barrett_sim {

  class BarrettSimManager;

  /** \brief State structure for a fake WAM device
   *
   */
  template<size_t DOF>
    class WamSimDevice : public oro_barrett_interface::WamDevice<DOF>
  {
  public:

    // TODO: Switch to oro_barrett_msgs
    enum RunMode {
      IDLE = 0,
      RUN = 1,
      ESTOP = 2
    };

    /** \brief Construct a low-level WAM interface and extract joint information from
     * the URDF.
     */
    WamSimDevice(
        RTT::Service::shared_ptr parent_service,
        const urdf::Model &urdf_model,
        const std::string &urdf_prefix,
        std::vector<gazebo::physics::JointPtr> gazebo_joints) :
      oro_barrett_interface::WamDevice<DOF>(
          parent_service,
          urdf_model,
          urdf_prefix),
      run_mode(IDLE),
      gazebo_joints_(gazebo_joints),
      raw_joint_position(Eigen::Matrix<double,DOF,1>::Zero(DOF)),
      raw_joint_velocity(Eigen::Matrix<double,DOF,1>::Zero(DOF))
    {
      this->status_msg.safety_mode.value = oro_barrett_msgs::SafetyMode::ACTIVE;
      this->status_msg.run_mode.value = oro_barrett_msgs::RunMode::IDLE;
      this->status_msg.homed = true;
    }

    virtual void initialize()
    {
      RTT::log(RTT::Info) << "Initializing simulated WAM." << RTT::endlog();
    }

    virtual void run()
    {
      RTT::log(RTT::Info) << "Running simulated WAM." << RTT::endlog();
      if(run_mode != RUN) {
        run_mode = RUN;
      }
    }

    virtual void idle()
    {
      RTT::log(RTT::Info) << "Idling simulated WAM." << RTT::endlog();
      run_mode = IDLE;
    }

    virtual void estop()
    {
      run_mode = ESTOP;
    }

    void readSim(ros::Time time, RTT::Seconds period)
    {
      if(period < 1.0E-5) {
          return;
      }
      // Get state from gazebo joints
      const double rc = 1.0/(2.0*M_PI*this->velocity_cutoff_frequency);
      const double a = period / (rc + period);

      for(unsigned j=0; j < DOF; j++) {
        double pos = (1.0-a)*raw_joint_position[j] + (a)*gazebo_joints_[j]->GetAngle(0).Radian();
        double vel = (pos - raw_joint_position[j]) / period;
        if(vel > 10 || period < 1E-6) {
          vel = 0;
        }
        raw_joint_position[j] = pos;
        raw_joint_velocity[j] = vel;
        // DO NOT DO THIS: raw_joint_velocity[j] = gazebo_joints_[j]->GetVelocity(0);
        // Known error that OSRF doesn't care about: https://bitbucket.org/osrf/gazebo/issues/622/joint-velocities-are-high-when-the-joint#
      }
    }

    void writeSim(ros::Time time, RTT::Seconds period)
    {
      switch(run_mode) {
        case RUN:
          // Write command to gazebo joints
          for(unsigned int j=0; j < this->gazebo_joints_.size() && j < this->joint_effort.size(); j++) {
            this->gazebo_joints_[j]->SetForce(0,this->joint_effort[j]);
          }
          break;

        case IDLE:
          // Write command to gazebo joints
          for(unsigned int j=0; j < this->gazebo_joints_.size() && j < this->joint_effort.size(); j++) {
            this->gazebo_joints_[j]->SetForce(0,0.0);
          }
          break;
      };
    }

    virtual void readDevice(ros::Time time, RTT::Seconds period)
    {
      // Exponentially smooth velocity
      this->joint_velocity = raw_joint_velocity;

      // Store position
      this->joint_position = raw_joint_position;

      // Write to data ports
      this->joint_position_out.write(this->joint_position);
      this->joint_velocity_out.write(this->joint_velocity);

      // Publish state to ROS
      if(this->joint_state_throttle.ready(0.02))
      {
        // Update the joint state message
        this->joint_state.header.stamp = rtt_rosclock::host_now();
        this->joint_state.name = this->joint_names;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.position.data(),DOF) = this->joint_position;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.velocity.data(),DOF) = this->joint_velocity;
        Eigen::Map<Eigen::VectorXd>(this->joint_state.effort.data(),DOF) = this->joint_effort;

        // Publish
        this->joint_state_out.write(this->joint_state);

        this->status_msg.run_mode.value = this->run_mode;
        this->status_out.write(this->status_msg);
      }
    }

    virtual void writeDevice(ros::Time time, RTT::Seconds period)
    {
      // Check if the effort command port is connected
      if(this->joint_effort_in.connected()) {
        // Read newest command from data ports
        Eigen::VectorXd joint_effort_tmp(DOF);
        bool new_effort_cmd = this->joint_effort_in.readNewest(joint_effort_tmp) == RTT::NewData;

        // Set the command to zero if there's no new command
        if(!new_effort_cmd) {
          this->joint_effort.setZero();
          return;
        }

        // Make sure the effort command is the right size
        if(joint_effort_tmp.size() == (unsigned)DOF) {
          this->joint_effort_raw = joint_effort_tmp;
        } else {
          this->joint_effort_raw.setZero();
        }

      } else {
        // Not connected, zero the command
        this->joint_effort_raw.resize(DOF);
        this->joint_effort_raw.setZero();
      }

      switch(run_mode) {
        case RUN:
          this->joint_effort = this->joint_effort_raw;
          break;

        case IDLE:
          break;
      };


      // Check effort limits
      for(size_t i=0; i<DOF; i++) {
        // Check if the joint effort is too high
        if(std::abs(this->joint_effort(i)) > this->warning_fault_ratio*this->joint_effort_limits[i]) {
          if(this->warning_count[i] % 1000 == 0) {
            // This warning can kill heartbeats
            RTT::log(RTT::Warning) << "Commanded torque (" << this->joint_effort(i)
              << ") of joint (" << i << ") is within "<<(1.0-this->warning_fault_ratio) * 100.0
              <<"\% of safety limit: " << this->joint_effort_limits[i] << RTT::endlog();
          }
          this->warning_count[i]++;

          // Check for fault
          if(std::abs(this->joint_effort(i)) > this->joint_effort_limits[i]) {
            RTT::log(RTT::Error) << "Commanded torque (" << this->joint_effort(i)
              << ") of joint (" << i << ") has exceeded safety limit: "
              << this->joint_effort_limits[i] << RTT::endlog();
            this->joint_effort.setZero();
            if(run_mode == RUN) {
              //this->parent_service_->getOwner()->error();
              return;
            }
          }
        }
      }

      // Save the last effort command
      this->joint_effort_last = this->joint_effort;

      // Pass along commanded effort for anyone who cares
      this->joint_effort_out.write(this->joint_effort);
    }

  protected:
    //! libbarrett Interface
    RunMode run_mode;

    //! Vector of gazebo joint references
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;

    // Simulation data buffers
    Eigen::Matrix<double,DOF,1>
      raw_joint_position,
      raw_joint_velocity;

  };

}

#endif // ifndef __ORO_BARRETT_SIM_WAM_SIM_DEVICE
