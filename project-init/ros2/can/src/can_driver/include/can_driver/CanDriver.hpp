#pragma once

#include <chrono>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <mutex>

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
	std::mutex mtx_;
	int sock_ = -1;

	std::string interface_name_;
	int reconnect_interval_ms_;
	std::chrono::steady_clock::time_point last_reconnect_attempt_;

	bool openSocket();
	void closeSocket();
	bool shouldRetryReconnect();
};