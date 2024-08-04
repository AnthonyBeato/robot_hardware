#include "robot_hardware/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_hardware{
    hardware_interface::CallbackReturn RobotHardware::on_init(const hardware_interface::HardwareInfo & info){
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "PID values not supplied, using defaults.");
        }
        

        wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // robotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveMSP432Hardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveMSP432Hardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveMSP432Hardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveMSP432Hardware"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveMSP432Hardware"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotHardware::on_cleanup(const rclcpp_lifecycle::State &  /*previous_state*/){
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Cleaning up ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Successfully cleaned up!");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotHardware::on_configure(const rclcpp_lifecycle::State &){
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Configuring ...please wait...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotHardware::on_activate(const rclcpp_lifecycle::State &  /*previous_state*/){
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Activating ...please wait...");
        if (!comms_.connected())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (cfg_.pid_p > 0)
        {
            comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
        }
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Successfully activated!");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotHardware::on_deactivate(const rclcpp_lifecycle::State &){
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432Hardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotHardware::read(const rclcpp::Time &, const rclcpp::Duration & period){
        if (!comms_.connected()){
            return hardware_interface::return_type::ERROR;
        }

        comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

        double delta_seconds = period.seconds();

        double pos_prev = wheel_l_.pos;
        wheel_l_.pos = wheel_l_.calc_enc_angle();
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

        pos_prev = wheel_r_.pos;
        wheel_r_.pos = wheel_r_.calc_enc_angle();
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardware::write(const rclcpp::Time &, const rclcpp::Duration &){
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
        int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
        comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotHardware, hardware_interface::SystemInterface)