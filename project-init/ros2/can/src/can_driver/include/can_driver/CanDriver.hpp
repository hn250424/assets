#pragma once

#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <chrono>
#include <iostream>

class CanDriver {
public:
    CanDriver();
    ~CanDriver();

    // Opens CAN interface with auto-reconnection.
    // Returns false on initial failure, but driver will automatically retry
    // during read()/write() calls at the specified interval.
    // Usage: Call open() once, then continuously call read()/write() in a loop.
    bool open(const std::string& name, int reconnect_interval_ms = 1000);
    void close();
    
    bool read(can_frame& frame);
    bool write(const can_frame& frame);

    // bool isConnected() const { return sock_ >= 0; }
    
private:
    int sock_ = -1;

    std::string interface_name_;
    int reconnect_interval_ms_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_;

    bool openSocket();
    void closeSocket();
    bool shouldRetryReconnect();
};