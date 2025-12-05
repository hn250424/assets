#include "can_driver/CanDriver.hpp"

CanDriver::CanDriver() {
    
}

CanDriver::~CanDriver() {
    close();
}

bool CanDriver::open(const std::string& name) {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) return false;

    ifreq ifr{};
    std::strcpy(ifr.ifr_name, name.c_str());
    int ifindex_result = ioctl(sock_, SIOCGIFINDEX, &ifr);
    if (ifindex_result < 0) return false;

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int socket_bind_ret = bind(sock_, (struct sockaddr*)&addr, sizeof(addr));
    if (socket_bind_ret < 0) {
        close();
        return false;
    }
     
    fcntl(sock_, F_SETFL, O_NONBLOCK);
    return true;
}

void CanDriver::close() {
    if (sock_ >= 0) {
        ::close(sock_);
    }
}

bool CanDriver::read(can_frame& frame) {
    if (sock_ < 0) return false;
    int nbytes = ::read(sock_, &frame, sizeof(frame));
    return nbytes > 0;
}

bool CanDriver::write(const can_frame& frame) {
    if (sock_ < 0) return false;
    int nbytes = ::write(sock_, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
}
