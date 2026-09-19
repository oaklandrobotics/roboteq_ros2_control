#ifndef ROBOTEQ_HARDWARE_INTERFACE_HPP
#define ROBOTEQ_HARDWARE_INTERFACE_HPP

// Internal headers
#include "roboteq_ros2_control/socket_can.hpp"
#include "roboteq_ros2_control/can_helpers.hpp"
#include "roboteq_ros2_control/canopen_enums.hpp"
#include "roboteq_ros2_control/canopen_sdo.hpp"
#include "roboteq_ros2_control/roboteq_object_dictionary.hpp"

// ROS2 headers
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace roboteq_ros2_control
{
  struct Axis
  {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, double gear_ratio) : can_intf_(can_intf), node_id_(node_id), gear_ratio_(gear_ratio) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);
    bool send_can_msg(const can_frame& frame) const {
      return can_intf_->send_can_frame(frame);
    }

    SocketCanIntf* can_intf_;
    uint8_t node_id_;
    
    double gear_ratio_;

    // Commands (ros2_control => Roboteq)
    double vel_setpoint_ = 0.0f; // [rad/s]
    // double pos_setpoint_ = 0.0f; // [rad] // TODO: Future implementation
    // double torque_setpoint_ = 0.0f; // [Nm] // TODO: Future implementation

    // State (Roboteq => ros2_control)
    double vel_estimate_ = 0.0; // [rad/s]
    double pos_estimate_ = 0.0; // [rad]
    // double torque_target_ = NAN; // [Nm] // TODO: Future implementation
    // double torque_estimate_ = NAN; // [Nm] // TODO: Future implementation

    bool motor_enabled_ = false;
    bool faulted_ = false;
    bool heartbeat_seen_ = false;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // bool pos_input_enabled_ = false; // TODO: Future implementation
    // bool vel_input_enabled_ = false; // TODO: Future implementation
    // bool torque_input_enabled_ = false; // TODO: Future implementation
  };

  class RoboteqHardwareInterface final : public hardware_interface::SystemInterface
  {
    public:
      using return_type = hardware_interface::return_type;
      using State = rclcpp_lifecycle::State;

      // ROS2 control transitions
      CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
      CallbackReturn on_configure(const State& previous_state) override;
      CallbackReturn on_cleanup(const State& previous_state) override;
      CallbackReturn on_activate(const State& previous_state) override;
      CallbackReturn on_deactivate(const State& previous_state) override;
      CallbackReturn on_error(const State& previous_state) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
      return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

      void reinitialize();
      void estop();

    private:
      void on_can_msg(const can_frame& frame);

      EpollEventLoop event_loop_;
      std::vector<Axis> axes_;
      std::string can_intf_name_;
      SocketCanIntf can_intf_;
      rclcpp::Time timestamp_;

      std::atomic<bool> estop_active_ = false;

      // The ratio for our current setup is 7.3:1
      double gear_ratio_ = 7.3;

      // For various ROS2 services
      rclcpp::executors::SingleThreadedExecutor service_executor_;
      std::shared_ptr<rclcpp::Node> service_node_;
      std::thread spin_thread_;

      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinit_srv_;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  };
} // namespace roboteq_ros2_control

#endif