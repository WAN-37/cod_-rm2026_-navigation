#include "uart_transporter.hpp"
#include "uart_transporter.cpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

namespace {
std::atomic_bool g_running{true};

void handleSignal(int)
{
  g_running = false;
}

uint8_t crc8Calculate(const uint8_t * data, size_t len)
{
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = static_cast<uint8_t>((crc << 1U) ^ 0x31U);
      } else {
        crc = static_cast<uint8_t>(crc << 1U);
      }
    }
  }
  return crc;
}
}

int main(int argc, char * argv[])
{
  const char * device = "/dev/ttyACM0";
  int baudrate = 115200;
  int interval_ms = 100;

  if (argc > 1) {
    device = argv[1];
  }
  if (argc > 2) {
    interval_ms = std::stoi(argv[2]);
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  uint8_t packet[23];
  const uint8_t protocol_cmd_id = 0xFF;
  const uint8_t frame_tail = 0x0D;
  const float roll = 1.0F;
  const float pitch = 2.0F;
  const float yaw = 3.0F;

  packet[0] = protocol_cmd_id;
  std::memcpy(&packet[1], &roll, sizeof(float));
  std::memcpy(&packet[5], &pitch, sizeof(float));
  std::memcpy(&packet[9], &yaw, sizeof(float));
  packet[13] = crc8Calculate(packet, 13);
  packet[14] = 1;
  packet[15] = 1;
  packet[16] = 1;
  packet[17] = 1;
  std::memcpy(&packet[18], &yaw, sizeof(float));
  packet[22] = frame_tail;

  UartTransporter uart(device, baudrate);
  if (!uart.open() || !uart.isOpen()) {
    std::cerr << "Failed to open UART " << device;
    if (!uart.errorMessage().empty()) {
      std::cerr << ": " << uart.errorMessage();
    }
    std::cerr << std::endl;
    return 1;
  }

  std::cout << "Start sending test packet to " << device
            << " every " << interval_ms << " ms" << std::endl;

  while (g_running) {
    int written = uart.writeBuffer(packet, sizeof(packet));
    if (written != static_cast<int>(sizeof(packet))) {
      std::cerr << "Write failed, bytes written: " << written << std::endl;
      uart.close();
      return 1;
    }

    std::cout << "sent:";
    for (uint8_t byte : packet) {
      std::cout << " 0x" << std::hex << std::uppercase
                << static_cast<int>(byte);
    }
    std::cout << std::dec << std::nouppercase << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }

  uart.close();
  std::cout << "Stopped." << std::endl;
  return 0;
}
