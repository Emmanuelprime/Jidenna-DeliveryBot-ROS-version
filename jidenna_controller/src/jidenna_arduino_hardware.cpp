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
// on_init — read URDF <hardware><param> values, size vectors, verify
//           each joint exposes a velocity command interface.
// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create a persistent clock for RCLCPP_*_THROTTLE macros.
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  // Safe param reader: value is a string; unknown key -> default
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

  stop_reader_ = false;
  reader_thread_ = std::thread(&JidennaArduinoHardware::reader_loop, this);

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
  send_command(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
hardware_interface::CallbackReturn
JidennaArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------ //
// Export interfaces — controller_manager wires these into the
// hardware_interface::ResourceManager.
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
// read — called every controller_manager cycle.
// Integrates joint positions from the latest measured velocities.
// ------------------------------------------------------------------ //
hardware_interface::return_type
JidennaArduinoHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (have_data_)
  {
    double dt = period.seconds();
    if (dt > 0.0 && dt < 0.5)
    {
      hw_positions_[0] += hw_velocities_[0] * dt;
      hw_positions_[1] += hw_velocities_[1] * dt;
    }
  }
  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------ //
// write — called every controller_manager cycle.
// Converts joint angular velocities (rad/s) to (v, w) and sends text.
// ------------------------------------------------------------------ //
hardware_interface::return_type
JidennaArduinoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double vL = hw_commands_[0];   // left wheel rad/s
  double vR = hw_commands_[1];   // right wheel rad/s

  double v     = (vL + vR) * 0.5 * wheel_radius_;               // m/s
  double omega = (vR - vL) * wheel_radius_ / wheel_sep_;        // rad/s
  

  if (v >  v_max_) v =  v_max_;
  if (v < -v_max_) v = -v_max_;
  if (omega >  w_max_) omega =  w_max_;
  if (omega < -w_max_) omega = -w_max_;

  if (!send_command(v, omega))
  {
    // Throttled: at most once every 2 s
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLogger), *clock_,
      2000, "Serial write failed");
  }

  last_cmd_time_ = std::chrono::steady_clock::now();
  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------ //
// Serial helpers — POSIX, no external dependency
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
  tty.c_cc[VTIME] = 1;   // 100 ms read timeout

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
//
// CSV format (from the Arduino firmware):
//   x,y,th,vL,vR,wL,wR,bat,temp,fb_age,wd,or,imu_yaw
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

      // Skip the Arduino boot banner: "x,y,th,..."
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
        try
        {
          vals.push_back(std::stod(tok));
        }
        catch (...)
        {
          ok = false;
          break;
        }
      }

      if (ok && vals.size() == 13)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_x_  = vals[0];
        robot_y_  = vals[1];
        robot_th_ = vals[2];
        // vals[3]=vL (m/s), vals[4]=vR (m/s) — not stored; hw velocities are rad/s
        hw_velocities_[0] = vals[5];   // wL rad/s
        hw_velocities_[1] = vals[6];   // wR rad/s
        bat_v_    = vals[7];
        temp_c_   = vals[8];
        // vals[9]=fb_age, vals[10]=wd, vals[11]=or_count
        imu_yaw_  = vals[12];
        have_data_ = true;
      }

      line.clear();
    }
    else if (c != '\r')
    {
      line.push_back(c);
      if (line.size() > 128) line.clear();   // resync on garbage
    }
  }
}

}  // namespace jidenna_controller


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(jidenna_controller::JidennaArduinoHardware,
                       hardware_interface::SystemInterface)