#ifndef JIDENNA_CONTROLLER__JIDENNA_ARDUINO_HARDWARE_HPP_
#define JIDENNA_CONTROLLER__JIDENNA_ARDUINO_HARDWARE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jidenna_controller
{

class JidennaArduinoHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JidennaArduinoHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial I/O
  bool open_serial();
  void close_serial();
  void reader_loop();
  bool send_command(double v, double w);

  // Config
  std::string port_;
  int         baud_         = 115200;
  double      v_max_        = 0.8;
  double      w_max_        = 3.0;
  double      wheel_radius_ = 0.0825;
  double      wheel_sep_    = 0.521;
  int         serial_fd_    = -1;

  // Joint state (must match URDF joint names)
  std::vector<std::string> joint_names_;
  std::vector<double> hw_commands_;    // [left, right] rad/s (commanded)
  std::vector<double> hw_positions_;   // integrated (fallback)
  std::vector<double> hw_velocities_;  // [left, right] rad/s (measured)

  // Telemetry (populated by reader thread)
  std::mutex  data_mutex_;
  double      robot_x_   = 0.0;
  double      robot_y_   = 0.0;
  double      robot_th_  = 0.0;
  double      bat_v_     = 0.0;
  double      temp_c_    = 0.0;
  double      imu_yaw_   = 0.0;
  bool        have_data_ = false;

  // Reader thread control
  std::thread       reader_thread_;
  std::atomic<bool> stop_reader_{false};

  // Throttle clock for RCLCPP_*_THROTTLE macros
  rclcpp::Clock::SharedPtr clock_;

  std::chrono::steady_clock::time_point last_cmd_time_;
};

}  // namespace jidenna_controller

#endif  // JIDENNA_CONTROLLER__JIDENNA_ARDUINO_HARDWARE_HPP_