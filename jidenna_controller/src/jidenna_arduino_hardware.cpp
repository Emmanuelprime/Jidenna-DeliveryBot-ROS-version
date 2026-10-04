#include "jidenna_controller/jidenna_arduino_hardware.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jidenna_controller
{

static constexpr const char * kLogger = "JidennaArduinoHardware";

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // System clock — reflects wall time, safe from any thread.
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  auto get_param = [&](const std::string & name, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(name);
    return it != info_.hardware_parameters.end() ? it->second : def;
  };

  try
  {
    port_         = get_param("port", "/dev/ttyUSB0");
    baud_         = std::stoi(get_param("baud", "115200"));
    v_max_        = std::stod(get_param("v_max", "0.8"));
    w_max_        = std::stod(get_param("w_max", "3.0"));
    wheel_radius_ = std::stod(get_param("wheel_radius", "0.0825"));
    wheel_sep_    = std::stod(get_param("wheel_separation", "0.521"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Bad hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);

  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);

    bool has_vel = false;
    for (const auto & ci : joint.command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_vel = true;
        break;
      }
    }
    if (!has_vel)
    {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
        "Joint '%s' must expose a velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ----- IMU publisher node -----
  imu_node_ = std::make_shared<rclcpp::Node>("jidenna_imu_publisher");
  imu_pub_  = imu_node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

  RCLCPP_INFO(rclcpp::get_logger(kLogger),
    "Initialized: port=%s baud=%d v_max=%.2f w_max=%.2f r=%.3f L=%.3f",
    port_.c_str(), baud_, v_max_, w_max_, wheel_radius_, wheel_sep_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  send_command(0.0, 0.0);

  // Reset yaw-rate tracking so the first CSV sample doesn't produce a huge spike
  prev_imu_yaw_valid_ = false;

  stop_reader_ = false;
  reader_thread_ = std::thread(&JidennaArduinoHardware::reader_loop, this);

  stop_imu_ = false;
  imu_thread_ = std::thread(&JidennaArduinoHardware::imu_publish_loop, this);

  last_cmd_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_reader_ = true;
  if (reader_thread_.joinable())
  {
    reader_thread_.join();
  }

  stop_imu_ = true;
  if (imu_thread_.joinable())
  {
    imu_thread_.join();
  }

  send_command(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_serial();
  imu_pub_.reset();
  imu_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
std::vector<hardware_interface::StateInterface>
JidennaArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    ifaces.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  ifaces.emplace_back("imu_sensor", "orientation.x",      &imu_orientation_[0]);
  ifaces.emplace_back("imu_sensor", "orientation.y",      &imu_orientation_[1]);
  ifaces.emplace_back("imu_sensor", "orientation.z",      &imu_orientation_[2]);
  ifaces.emplace_back("imu_sensor", "orientation.w",      &imu_orientation_[3]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.x", &imu_angular_velocity_[0]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.y", &imu_angular_velocity_[1]);
  ifaces.emplace_back("imu_sensor", "angular_velocity.z", &imu_angular_velocity_[2]);

  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
JidennaArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    ifaces.emplace_back(info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ifaces;
}

// ------------------------------------------------------------------ //
hardware_interface::return_type
JidennaArduinoHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  double dt = period.seconds();

  if (have_data_)
  {
    if (dt > 0.0 && dt < 0.5)
    {
      hw_positions_[0] += hw_velocities_[0] * dt;
      hw_positions_[1] += hw_velocities_[1] * dt;
    }

    // IMU orientation (quaternion) from yaw
    double yaw = imu_yaw_;
    imu_orientation_[0] = 0.0;
    imu_orientation_[1] = 0.0;
    imu_orientation_[2] = std::sin(yaw * 0.5);
    imu_orientation_[3] = std::cos(yaw * 0.5);

    // Yaw rate (imu_angular_velocity_[2]) is computed in reader_loop()
    // based on CSV arrival timing.
  }

  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------ //
hardware_interface::return_type
JidennaArduinoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double vL = hw_commands_[0];
  double vR = hw_commands_[1];

  double v     = (vL + vR) * 0.5 * wheel_radius_;
  double omega = (vR - vL) * wheel_radius_ / wheel_sep_;

  if (v >  v_max_) v =  v_max_;
  if (v < -v_max_) v = -v_max_;
  if (omega >  w_max_) omega =  w_max_;
  if (omega < -w_max_) omega = -w_max_;

  if (!send_command(v, omega))
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLogger), *clock_,
      2000, "Serial write failed");
  }

  last_cmd_time_ = std::chrono::steady_clock::now();
  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------ //
bool JidennaArduinoHardware::open_serial()
{
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
      "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "tcgetattr failed");
    return false;
  }

  cfmakeraw(&tty);

  speed_t speed = B115200;
  switch (baud_)
  {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger(kLogger),
        "Unsupported baud %d, using 115200", baud_);
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "tcsetattr failed");
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void JidennaArduinoHardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool JidennaArduinoHardware::send_command(double v, double w)
{
  if (serial_fd_ < 0) return false;

  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%.4f,%.4f\n", v, w);
  if (n <= 0) return false;

  ssize_t written = ::write(serial_fd_, buf, static_cast<size_t>(n));
  return written == n;
}

// ------------------------------------------------------------------ //
// Reader thread — parses CSV lines from Arduino.
// ------------------------------------------------------------------ //
void JidennaArduinoHardware::reader_loop()
{
  std::string line;
  line.reserve(128);
  char c;

  while (!stop_reader_)
  {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n <= 0) continue;

    if (c == '\n')
    {
      if (line.empty()) continue;

      if (line.rfind("x,", 0) == 0)
      {
        line.clear();
        continue;
      }

      std::stringstream ss(line);
      std::string tok;
      std::vector<double> vals;
      vals.reserve(13);

      bool ok = true;
      while (std::getline(ss, tok, ','))
      {
        try { vals.push_back(std::stod(tok)); }
        catch (...) { ok = false; break; }
      }

      if (ok && vals.size() == 13)
      {
        auto now_tp = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_x_  = vals[0];
        robot_y_  = vals[1];
        robot_th_ = vals[2];
        hw_velocities_[0] = vals[6];   // wL rad/s
        hw_velocities_[1] = vals[5];   // wR rad/s
        bat_v_    = vals[7];
        temp_c_   = vals[8];

        // ---- Yaw rate from CSV arrival timing ----
        double new_yaw = vals[12];
        if (prev_imu_yaw_valid_)
        {
          double dt = std::chrono::duration<double>(
                        now_tp - prev_imu_yaw_time_).count();
          if (dt > 1e-3 && dt < 0.5)
          {
            double dyaw = new_yaw - prev_imu_yaw_;
            while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
            imu_angular_velocity_[2] = dyaw / dt;
          }
        }
        prev_imu_yaw_       = new_yaw;
        prev_imu_yaw_time_  = now_tp;
        prev_imu_yaw_valid_ = true;

        imu_yaw_   = new_yaw;
        have_data_ = true;
      }

      line.clear();
    }
    else if (c != '\r')
    {
      line.push_back(c);
      if (line.size() > 128) line.clear();
    }
  }
}

// ------------------------------------------------------------------ //
// IMU publish thread — publishes /imu/data at 50 Hz.
// ------------------------------------------------------------------ //
void JidennaArduinoHardware::imu_publish_loop()
{
  rclcpp::Rate rate(50.0);
  while (!stop_imu_ && rclcpp::ok())
  {
    if (imu_pub_ && have_data_)
    {
      sensor_msgs::msg::Imu msg;
      // Use the RCL_SYSTEM_TIME clock — always returns current wall time.
      msg.header.stamp    = clock_->now();
      msg.header.frame_id = "imu_link";

      std::lock_guard<std::mutex> lock(data_mutex_);
      msg.orientation.x = imu_orientation_[0];
      msg.orientation.y = imu_orientation_[1];
      msg.orientation.z = imu_orientation_[2];
      msg.orientation.w = imu_orientation_[3];
      msg.angular_velocity.x = imu_angular_velocity_[0];
      msg.angular_velocity.y = imu_angular_velocity_[1];
      msg.angular_velocity.z = imu_angular_velocity_[2];

      msg.orientation_covariance[0] = -1.0;   // roll not provided
      msg.orientation_covariance[4] = -1.0;   // pitch not provided
      msg.orientation_covariance[8] = 0.05;   // yaw stddev ~0.22 rad

      msg.angular_velocity_covariance[0] = -1.0;
      msg.angular_velocity_covariance[4] = -1.0;
      msg.angular_velocity_covariance[8] = 0.01;   // yaw rate stddev

      msg.linear_acceleration_covariance[0] = -1.0;
      msg.linear_acceleration_covariance[4] = -1.0;
      msg.linear_acceleration_covariance[8] = -1.0;

      imu_pub_->publish(msg);
    }
    rate.sleep();
  }
}

}  // namespace jidenna_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jidenna_controller::JidennaArduinoHardware,
                       hardware_interface::SystemInterface)