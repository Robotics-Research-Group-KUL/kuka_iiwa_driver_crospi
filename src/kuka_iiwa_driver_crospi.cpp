#include "kuka_iiwa_driver_crospi/kuka_iiwa_driver_crospi.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {

#define NUM_JOINTS 7
#define MAX_RETRIES 3000

kuka_iiwa_driver_crospi::kuka_iiwa_driver_crospi(): 
    control_mode(ControlMode::ControlMode::IDLE),
    app(connection,client) ,
    iiwa_connected(false)

{
}

void kuka_iiwa_driver_crospi::construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    ip_address = jsonchecker->asString(config, "ip_address");
    
    // get fri_port from config
    fri_port = jsonchecker->asUInt(config, "fri_port");

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;

    constructPorts(NUM_JOINTS, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.
    
    setpoint_joint_vel_struct.data.resize(static_cast<int>(LBRState::NUMBER_OF_JOINTS), 0.0); //resize and initialize setpoint joint velocities to zero
    joint_pos_struct.data.resize(static_cast<int>(LBRState::NUMBER_OF_JOINTS), 0.0);

    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of kuka_iiwa_driver_crospi class with name: " << name << std::endl;

    first_commanding_active = false;

}

bool kuka_iiwa_driver_crospi::initialize()
{

    iiwa_connected = app.connect(fri_port, ip_address.c_str());
    // // TODO : Raise error if iiwa connected is false
    // // if (!iiwa_connected)
    // // {
    // //     fmt::print("Error: Could not connect to the robot\n");
    // //     return false;
    // // }

    // // TODO: Add timeout to the communication
    int num_retries = 0;
    while (client.current_session_state != MONITORING_READY)
    {
        if (num_retries > MAX_RETRIES)
        {
            fmt::print("Error: Could not connect to the robot\n");
            return false;
        }
        app.step(); //This is blocking
        client.getDiscreteState();
        client.getContinousState();
        num_retries++;
    }

    assert(joint_pos_struct.data.size() == static_cast<int>(LBRState::NUMBER_OF_JOINTS));

    for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
        joint_pos_struct.data[i] = client.meas_jnt_pos[i];
    }
    
    writeFeedbackJointPosition(joint_pos_struct);

    return true;
}

void kuka_iiwa_driver_crospi::update(volatile std::atomic<bool>& stopFlag)
{

    client.getContinousState();
	client.getDiscreteState();

    readSetpointJointVelocity(setpoint_joint_vel_struct);
    assert(setpoint_joint_vel_struct.data.size() == static_cast<int>(LBRState::NUMBER_OF_JOINTS));

    if (client.current_session_state == COMMANDING_ACTIVE)
    {
        for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
            client.cmd_jnt_pos[i] += setpoint_joint_vel_struct.data[i]*client.robotState().getSampleTime();
        }
        first_commanding_active = true;
    }
    else if (first_commanding_active)
    {
        std::cout << "WARNING: Robot not in COMMANDING_ACTIVE state, stopping the robot." << std::endl;
        stopFlag.store(true);
    }

    for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS;i++) {
        joint_pos_struct.data[i] = client.meas_jnt_pos[i];
    }
    writeFeedbackJointPosition(joint_pos_struct);

    app.step();
}

void kuka_iiwa_driver_crospi::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void kuka_iiwa_driver_crospi::on_activate() 
{


}

void kuka_iiwa_driver_crospi::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void kuka_iiwa_driver_crospi::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void kuka_iiwa_driver_crospi::finalize() {
    std::cout << "finalize() called =======================" << std::endl;
    app.disconnect();

}



kuka_iiwa_driver_crospi::~kuka_iiwa_driver_crospi() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::kuka_iiwa_driver_crospi, etasl::RobotDriver)
