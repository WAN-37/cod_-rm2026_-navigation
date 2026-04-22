#include "uart_transporter.hpp"
#include "uart_transporter.cpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber"),
    device_path_(this->declare_parameter<std::string>("device_path", "/dev/ttyACM0")),
    baudrate_(this->declare_parameter<int>("baudrate", 115200)),
    log_hex_payload_(this->declare_parameter<bool>("log_hex_payload", false)),
    stats_period_ms_(this->declare_parameter<int>("stats_period_ms", 1000)),
    uart_(device_path_, baudrate_)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));

    stats_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(stats_period_ms_),
      std::bind(&CmdVelSubscriber::logStats, this));
    last_stats_time_ = this->now();

    mcu_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("mcu_data", 10);

    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&CmdVelSubscriber::readSerialCallback, this));

    if (!ensureUartOpen()) {
      RCLCPP_WARN(
        this->get_logger(),
        "UART %s is not ready at startup, the node will keep retrying when messages arrive.",
        device_path_.c_str());
    }
  }

  ~CmdVelSubscriber() override
  {
    uart_.close();
  }

private:
  static std::string formatPacket(const std::array<uint8_t, 15> & packet)
  {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < packet.size(); ++i) {
      if (i != 0) {
        oss << ' ';
      }
      oss << "0x" << std::setw(2) << static_cast<int>(packet[i]);
    }
    return oss.str();
  }

  bool ensureUartOpen()
  {
    if (uart_.isOpen()) {
      return true;
    }

    if (!uart_.open()) {
      ++open_failures_;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to open UART %s: %s",
        device_path_.c_str(), uart_.errorMessage().c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "Opened UART %s at %d baud",
      device_path_.c_str(), baudrate_);
    return true;
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    ++received_msgs_;

    last_vx_ = static_cast<float>(msg->linear.x);
    last_vy_ = static_cast<float>(msg->linear.y);
    last_vz_ = static_cast<float>(msg->angular.z);

    if (!ensureUartOpen()) {
      return;
    }

    std::array<uint8_t, 15> packet{};
    packet[0] = 0xFF;
    std::memcpy(packet.data() + 1, &last_vx_, sizeof(float));
    std::memcpy(packet.data() + 5, &last_vy_, sizeof(float));
    std::memcpy(packet.data() + 9, &last_vz_, sizeof(float));
    packet[13] = 0x00;
    packet[14] = 0x0D;

    int written = uart_.writeBuffer(packet.data(), static_cast<int>(packet.size()));
    if (written != static_cast<int>(packet.size())) {
      ++write_failures_;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Serial write incomplete: wrote %d/%zu bytes to %s",
        written, packet.size(), device_path_.c_str());
      uart_.close();
      return;
    }

    ++sent_packets_;
    sent_bytes_ += static_cast<size_t>(written);

    if (log_hex_payload_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 200,
        "tx packet: vx=%.3f vy=%.3f vz=%.3f raw=[%s]",
        last_vx_, last_vy_, last_vz_, formatPacket(packet).c_str());
    }
  }

  void readSerialCallback()
  {
    if (!uart_.isOpen()) {
      return;
    }

    int bytes_available = 0;
    if (ioctl(uart_.fd_, FIONREAD, &bytes_available) < 0 || bytes_available <= 0) {
      return;
    }

    std::vector<uint8_t> tmp(bytes_available);
    int n = uart_.read(tmp.data(), tmp.size());
    if (n <= 0) {
      return;
    }

    rx_buf_.insert(rx_buf_.end(), tmp.begin(), tmp.begin() + n);

    static constexpr size_t FRAME_LEN = 15;
    static constexpr uint8_t FRAME_HEAD = 0xFF;
    static constexpr uint8_t FRAME_TAIL = 0x0D;

    while (rx_buf_.size() >= FRAME_LEN) {
      auto it = std::find(rx_buf_.begin(), rx_buf_.end(), FRAME_HEAD);
      if (it == rx_buf_.end()) {
        rx_buf_.clear();
        break;
      }

      if (it != rx_buf_.begin()) {
        rx_buf_.erase(rx_buf_.begin(), it);
      }

      if (rx_buf_.size() < FRAME_LEN) {
        break;
      }

      if (rx_buf_[14] != FRAME_TAIL) {
        ++rx_frame_errors_;
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }

      float x, y, z;
      std::memcpy(&x, rx_buf_.data() + 1, sizeof(float));
      std::memcpy(&y, rx_buf_.data() + 5, sizeof(float));
      std::memcpy(&z, rx_buf_.data() + 9, sizeof(float));

      last_rx_x_ = x;
      last_rx_y_ = y;
      last_rx_z_ = z;
      ++rx_valid_frames_;

      auto point_msg = geometry_msgs::msg::PointStamped();
      point_msg.header.stamp = this->now();
      point_msg.header.frame_id = "base_link";
      point_msg.point.x = static_cast<double>(x);
      point_msg.point.y = static_cast<double>(y);
      point_msg.point.z = static_cast<double>(z);
      mcu_pub_->publish(point_msg);

      if (log_hex_payload_) {
        std::array<uint8_t, 15> pkt;
        std::copy(rx_buf_.begin(), rx_buf_.begin() + 15, pkt.begin());
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 200,
          "rx packet: x=%.3f y=%.3f z=%.3f raw=[%s]",
          x, y, z, formatPacket(pkt).c_str());
      }

      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + FRAME_LEN);
    }

    if (rx_buf_.size() > 1024) {
      rx_buf_.clear();
    }
  }

  void logStats()
  {
    const auto now = this->now();
    const double dt = (now - last_stats_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    const auto rx_delta = received_msgs_ - last_received_msgs_;
    const auto tx_delta = sent_packets_ - last_sent_packets_;
    const auto bytes_delta = sent_bytes_ - last_sent_bytes_;

    const double rx_hz = static_cast<double>(rx_delta) / dt;
    const double tx_hz = static_cast<double>(tx_delta) / dt;
    const double kbps = static_cast<double>(bytes_delta) * 8.0 / dt / 1000.0;

    const auto rx_frame_delta = rx_valid_frames_ - last_rx_valid_frames_;
    const double rx_frame_hz = static_cast<double>(rx_frame_delta) / dt;

    RCLCPP_INFO(
      this->get_logger(),
      "serial stats: rx=%.1f Hz tx=%.1f Hz mcu_rx=%.1f Hz rate=%.2f kbps total_ok=%zu write_fail=%zu open_fail=%zu frame_err=%zu last_cmd=[%.3f, %.3f, %.3f] last_mcu=[%.3f, %.3f, %.3f]",
      rx_hz, tx_hz, rx_frame_hz, kbps, sent_packets_, write_failures_, open_failures_, rx_frame_errors_,
      last_vx_, last_vy_, last_vz_, last_rx_x_, last_rx_y_, last_rx_z_);

    last_stats_time_ = now;
    last_received_msgs_ = received_msgs_;
    last_sent_packets_ = sent_packets_;
    last_sent_bytes_ = sent_bytes_;
    last_rx_valid_frames_ = rx_valid_frames_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr mcu_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  std::string device_path_;
  int baudrate_;
  bool log_hex_payload_;
  int stats_period_ms_;
  UartTransporter uart_;
  size_t received_msgs_{0};
  size_t sent_packets_{0};
  size_t sent_bytes_{0};
  size_t write_failures_{0};
  size_t open_failures_{0};
  size_t last_received_msgs_{0};
  size_t last_sent_packets_{0};
  size_t last_sent_bytes_{0};
  float last_vx_{0.0F};
  float last_vy_{0.0F};
  float last_vz_{0.0F};
  std::vector<uint8_t> rx_buf_;
  size_t rx_valid_frames_{0};
  size_t rx_frame_errors_{0};
  size_t last_rx_valid_frames_{0};
  float last_rx_x_{0.0F};
  float last_rx_y_{0.0F};
  float last_rx_z_{0.0F};
  rclcpp::Time last_stats_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
