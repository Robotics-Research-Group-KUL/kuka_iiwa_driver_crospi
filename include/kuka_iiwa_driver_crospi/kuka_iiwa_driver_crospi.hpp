#pragma once

#include <string>

// KUKA FRI SDK
#include "kukaiiwa_client.hpp"
#include <friUdpConnection.h>
#include <friClientApplication.h>

#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"


namespace etasl {

class kuka_iiwa_driver_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<kuka_iiwa_driver_crospi> SharedPtr;


    private:
        
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        ControlMode::ControlMode control_mode;

        std::vector<double> joint_pos;
        std::string ip_address;
        unsigned int fri_port;

        ///FRI-related
        iiwaClient         client;
        UdpConnection     connection;
        ClientApplication app;
        bool iiwa_connected;
        bool first_commanding_active;

        // Fields for storing the data to be sent/received to/from the robot
        robotdrivers::DynamicJointDataField setpoint_joint_vel_struct;
        robotdrivers::DynamicJointDataField joint_pos_struct;

    public:
        kuka_iiwa_driver_crospi();

        virtual void construct(std::string robot_name,
                    const Json::Value& config,
                    std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        // virtual const std::string& getName() const override;

        virtual ~kuka_iiwa_driver_crospi();
};

} // namespace etasl
