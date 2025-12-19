#include "can_driver/CanDriver.hpp"

CanDriver::CanDriver() {
}

CanDriver::~CanDriver() {
	close();
}

bool CanDriver::open(const std::string& name, int reconnect_interval_ms) {
	interface_name_ = name;
	reconnect_interval_ms_ = reconnect_interval_ms;
	last_reconnect_attempt_ = std::chrono::steady_clock::now();

	return openSocket();
}

void CanDriver::close() {
	closeSocket();
}

bool CanDriver::openSocket() {
	printf("[CanDriver] Try openSocket()..\n");

	closeSocket();

	// Protocol Family (PF_XXX)
	// PF_CAN        : CAN Bus
	// PF_INET       : IPv4
	// PF_INET6      : IPv6
	// PF_BLUETOOTH  : Bluetooth
	// PF_PACKET     : Raw Ethernet packet

	// SOCK_RAW      : Receive all frames, control directly.
	// SOCK_DGRAM    : Datagram like UDP.
	// SOCK_STREAM   : Stream like TCP.

	// CAN_RAW       : Receive all frames without filtering.
	// CAN_BCM       : Broadcast Manager, filter ID, periodic transmission.
	sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sock_ < 0)
		return false;

	// ifreq        : interface request structure for network interface operations.
	// ioctl        : I/O control, system call to control device/driver.
	// SIOCGIFINDEX : Command code to get interface index (name -> number).
	// After ioctl(), ifr.ifr_ifindex contains the index number.
	ifreq ifr{};
	std::strcpy(ifr.ifr_name, interface_name_.c_str());
	int ifindex_result = ioctl(sock_, SIOCGIFINDEX, &ifr);
	if (ifindex_result < 0) {
		printf("[CanDriver] Failed to find interface %s\n", interface_name_.c_str());
		closeSocket();
		return false;
	}

	// Prepare CAN address structure for binding to read/write on the interface.
	// AF: Protocol Family.
	// PF: Address Family.
	// Historically they were meant to be distinct,
	// but in practice they represent the same values.
	// By convention, PF is used for sockets and AF is used for addresses.
	sockaddr_can addr{};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	// bind() is a system call that accepts generic sockaddr* for all protocols.
	int socket_bind_ret = bind(sock_, (struct sockaddr*)&addr, sizeof(addr));
	if (socket_bind_ret < 0) {
		printf("[CanDriver] Failed to bind to %s (check permissions or interface state)\n", interface_name_.c_str());
		closeSocket();
		return false;
	}

	// Ready blocking socket with set timeout.
	// socket init config: block and wait, so exclue <fcntl(sock_, F_SETFL, O_NONBLOCK);> for intercept not polling.
	//
	// SO_RCVTIMEO: Set a timeout for receiving data.
	// This prevents read() from blocking forever when can interface down.
	// Even in blocking mode, read() will return -1 after 1 second
	// if no data arrives, setting errno to EAGAIN or EWOULDBLOCK.
	//
	// SOL_SOCKET: The option's level is typical settings.
	// SO_RCVTIMEO: 'S'ocket 'O'ption - 'R'e'C'ei'V'e 'TIMEO'ut.
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

	return true;
}

void CanDriver::closeSocket() {
	if (sock_ >= 0) {
		printf("[CanDriver] Closing socket fd= %d\n", sock_);
		::close(sock_);
		sock_ = -1;
	}
}

bool CanDriver::read(can_frame& frame) {
	// If socket is not connected, try reconncecting.
	// Prepare for next attempt.
	// This read() fails, but will retry in next polling.
	if (sock_ < 0) {
		if (shouldRetryReconnect()) {
			openSocket();
		}

		return false;
	}

	// If read() fails, 'errno' is set automatically by the system.
	int nbytes = ::read(sock_, &frame, sizeof(frame));

	// CAN messages are atomic so always read a complete frame or nothing.
	// EAGAIN and EWOULDBLOCK mean 'no data available right now so try again later',
	// If nbytes is -1 and it's NOT these flags, it means it's not just a 'no data' situation
	// but a real communication error. In this case, we close and try to reopen the socket.
	if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		closeSocket();
		if (shouldRetryReconnect()) {
			openSocket();
		}
		return false;
	}

	return nbytes > 0;
}

bool CanDriver::write(const can_frame& frame) {
	if (sock_ < 0) {
		if (shouldRetryReconnect()) {
			openSocket();
		}
		return false;
	}

	int nbytes = ::write(sock_, &frame, sizeof(frame));

	if (nbytes < 0) {
		closeSocket();
		if (shouldRetryReconnect()) {
			openSocket();
		}
		return false;
	}

	return nbytes == sizeof(frame);
}

bool CanDriver::shouldRetryReconnect() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_attempt_).count();

	if (elapsed >= reconnect_interval_ms_) {
		last_reconnect_attempt_ = now;
		return true;
	}

	return false;
}