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

  uint8_t packet[15];
  packet[0] = 0xFF;
  std::memset(&packet[1], 0x01, 12);
  packet[13] = 0x00;
  packet[14] = 0x0D;

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
