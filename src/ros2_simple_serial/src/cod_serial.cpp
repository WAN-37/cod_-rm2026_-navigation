#include "uart_transporter.hpp"
#include "uart_transporter.cpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstring>

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    // 创建一个的数据包
    constexpr uint8_t header = 0xFF;
    constexpr uint8_t tail_high = 0x00;
    constexpr uint8_t tail_low = 0x0D;
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vz = msg->linear.z;

    // Convert the packet to a byte array
    uint8_t packet[15];
    packet[0] = header;
    std::memcpy(&packet[1], &vx, sizeof(float));
    std::memcpy(&packet[5], &vy, sizeof(float));
    std::memcpy(&packet[9], &vz, sizeof(float));

    // Add frame tail to the packet
    packet[13] = tail_high;
    packet[14] = tail_low;

    // Print packet data for debugging
    RCLCPP_INFO(this->get_logger(), "Packet data:");
    for (size_t i = 0; i < sizeof(packet); i++) {
      RCLCPP_INFO(this->get_logger(), "byte %zu: 0x%02X", i, packet[i]);
    }

    // Send the packet to the UART
    UartTransporter uart("/dev/ttyACM0", 115200);
    uart.open();
    if (uart.isOpen()) {
      RCLCPP_INFO(this->get_logger(),"Opened UART");
      uart.writeBuffer(packet, sizeof(packet));
      uart.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open UART");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
