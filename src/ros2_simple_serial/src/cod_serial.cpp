#include "uart_transporter.hpp"
#include "uart_transporter.cpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct __attribute__((packed)) DaohangAutoSendToNucData
{
  uint8_t frame_header;
  float roll;
  float pitch;
  float yaw;
  uint8_t check_byte;
  uint8_t is_recover;
  uint8_t self_status;
  uint8_t zone_status;
  uint8_t is_defence;
  float hp;
  uint8_t frame_tail;
};

static_assert(sizeof(DaohangAutoSendToNucData) == 23, "Unexpected RX frame size");

float unpackFloat(const void * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(float));
  return value;
}
}

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber"),
    device_path_(this->declare_parameter<std::string>("device_path", "/dev/ttyACM0")),
    baudrate_(this->declare_parameter<int>("baudrate", 115200)),
    log_hex_payload_(this->declare_parameter<bool>("log_hex_payload", false)),
    stats_period_ms_(this->declare_parameter<int>("stats_period_ms", 1000)),
    rx_protocol_cmd_id_(static_cast<uint8_t>(this->declare_parameter<int>("rx_protocol_cmd_id", 0xFF))),
    rx_frame_tail_(static_cast<uint8_t>(this->declare_parameter<int>("rx_frame_tail", 0x0D))),
    uart_(device_path_, baudrate_)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));

    switch_posture_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sentry_switch_posture_cmd", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        switch_state_ = static_cast<uint8_t>(msg->data);
      });

    confirm_revive_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sentry_confirm_revive_cmd", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        revive_confirm_ = static_cast<uint8_t>(msg->data);
      });

    buy_projectile_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sentry_buy_projectile_cmd", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        shoot_count_confirm_ = static_cast<uint8_t>(msg->data);
      });

    stats_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(stats_period_ms_),
      std::bind(&CmdVelSubscriber::logStats, this));
    last_stats_time_ = this->now();

    mcu_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("mcu_data", 10);

    // Serial receive is temporarily disabled.
    // read_timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(5),
    //   std::bind(&CmdVelSubscriber::readSerialCallback, this));

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
  static constexpr size_t kTxFrameLen = 18;
  static constexpr size_t kRxFrameLen = sizeof(DaohangAutoSendToNucData);
  static constexpr size_t kRxTailOffset = offsetof(DaohangAutoSendToNucData, frame_tail);

  template<size_t N>
  static std::string formatPacket(const std::array<uint8_t, N> & packet)
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

    std::array<uint8_t, kTxFrameLen> packet{};
    packet[0] = 0xFF;
    std::memcpy(packet.data() + 1, &last_vx_, sizeof(float));
    std::memcpy(packet.data() + 5, &last_vy_, sizeof(float));
    std::memcpy(packet.data() + 9, &last_vz_, sizeof(float));
    packet[13] = switch_state_;
    packet[14] = revive_confirm_;
    packet[15] = shoot_count_confirm_;
    packet[16] = 0x00;
    packet[17] = 0x0D;

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
    // if (!uart_.isOpen()) {
    //   return;
    // }
    //
    // int bytes_available = 0;
    // if (ioctl(uart_.fd_, FIONREAD, &bytes_available) < 0 || bytes_available <= 0) {
    //   return;
    // }
    //
    // std::vector<uint8_t> tmp(bytes_available);
    // int n = uart_.read(tmp.data(), tmp.size());
    // if (n <= 0) {
    //   return;
    // }
    //
    // rx_buf_.insert(rx_buf_.end(), tmp.begin(), tmp.begin() + n);
    //
    // while (rx_buf_.size() >= kRxFrameLen) {
    //   auto it = std::find(rx_buf_.begin(), rx_buf_.end(), rx_protocol_cmd_id_);
    //   if (it == rx_buf_.end()) {
    //     rx_buf_.clear();
    //     break;
    //   }
    //
    //   if (it != rx_buf_.begin()) {
    //     rx_buf_.erase(rx_buf_.begin(), it);
    //   }
    //
    //   if (rx_buf_.size() < kRxFrameLen) {
    //     break;
    //   }
    //
    //   if (rx_buf_[kRxTailOffset] != rx_frame_tail_) {
    //     ++rx_frame_errors_;
    //     rx_buf_.erase(rx_buf_.begin());
    //     continue;
    //   }
    //
    //   DaohangAutoSendToNucData frame{};
    //   std::memcpy(&frame, rx_buf_.data(), sizeof(frame));
    //
    //   const float roll = unpackFloat(&frame.roll);
    //   const float pitch = unpackFloat(&frame.pitch);
    //   const float yaw = unpackFloat(&frame.yaw);
    //   const float hp = unpackFloat(&frame.hp);
    //
    //   last_rx_roll_ = roll;
    //   last_rx_pitch_ = pitch;
    //   last_rx_yaw_ = yaw;
    //   last_rx_hp_ = hp;
    //   last_rx_is_recover_ = frame.is_recover;
    //   last_rx_self_status_ = frame.self_status;
    //   last_rx_zone_status_ = frame.zone_status;
    //   last_rx_is_defence_ = frame.is_defence;
    //   ++rx_valid_frames_;
    //
    //   auto point_msg = geometry_msgs::msg::PointStamped();
    //   point_msg.header.stamp = this->now();
    //   point_msg.header.frame_id = "base_link";
    //   point_msg.point.x = static_cast<double>(roll);
    //   point_msg.point.y = static_cast<double>(pitch);
    //   point_msg.point.z = static_cast<double>(yaw);
    //   mcu_pub_->publish(point_msg);
    //
    //   if (log_hex_payload_) {
    //     std::array<uint8_t, kRxFrameLen> pkt;
    //     std::copy(rx_buf_.begin(), rx_buf_.begin() + kRxFrameLen, pkt.begin());
    //     RCLCPP_INFO_THROTTLE(
    //       this->get_logger(), *this->get_clock(), 200,
    //       "rx packet: roll=%.3f pitch=%.3f yaw=%.3f hp=%.3f recover=%u self=%u zone=%u defence=%u raw=[%s]",
    //       roll, pitch, yaw, hp,
    //       static_cast<unsigned int>(frame.is_recover),
    //       static_cast<unsigned int>(frame.self_status),
    //       static_cast<unsigned int>(frame.zone_status),
    //       static_cast<unsigned int>(frame.is_defence),
    //       formatPacket(pkt).c_str());
    //   }
    //
    //   rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + kRxFrameLen);
    // }
    //
    // if (rx_buf_.size() > 1024) {
    //   rx_buf_.clear();
    // }
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
      "serial stats: rx=%.1f Hz tx=%.1f Hz mcu_rx=%.1f Hz rate=%.2f kbps total_ok=%zu write_fail=%zu open_fail=%zu frame_err=%zu last_cmd=[%.3f, %.3f, %.3f] last_mcu_rpy=[%.3f, %.3f, %.3f] last_mcu_status=[%u, %u, %u, %u] last_hp=%.3f",
      rx_hz, tx_hz, rx_frame_hz, kbps, sent_packets_, write_failures_, open_failures_, rx_frame_errors_,
      last_vx_, last_vy_, last_vz_, last_rx_roll_, last_rx_pitch_, last_rx_yaw_,
      static_cast<unsigned int>(last_rx_is_recover_),
      static_cast<unsigned int>(last_rx_self_status_),
      static_cast<unsigned int>(last_rx_zone_status_),
      static_cast<unsigned int>(last_rx_is_defence_),
      last_rx_hp_);

    last_stats_time_ = now;
    last_received_msgs_ = received_msgs_;
    last_sent_packets_ = sent_packets_;
    last_sent_bytes_ = sent_bytes_;
    last_rx_valid_frames_ = rx_valid_frames_;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr switch_posture_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr confirm_revive_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr buy_projectile_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr mcu_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  std::string device_path_;
  int baudrate_;
  bool log_hex_payload_;
  int stats_period_ms_;
  uint8_t rx_protocol_cmd_id_;
  uint8_t rx_frame_tail_;
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
  uint8_t switch_state_{0};
  uint8_t revive_confirm_{0};
  uint8_t shoot_count_confirm_{0};
  std::vector<uint8_t> rx_buf_;
  size_t rx_valid_frames_{0};
  size_t rx_frame_errors_{0};
  size_t last_rx_valid_frames_{0};
  float last_rx_roll_{0.0F};
  float last_rx_pitch_{0.0F};
  float last_rx_yaw_{0.0F};
  float last_rx_hp_{0.0F};
  uint8_t last_rx_is_recover_{0};
  uint8_t last_rx_self_status_{0};
  uint8_t last_rx_zone_status_{0};
  uint8_t last_rx_is_defence_{0};
  rclcpp::Time last_stats_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
