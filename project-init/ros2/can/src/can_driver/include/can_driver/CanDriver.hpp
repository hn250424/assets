#pragma once

#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <fcntl.h>

class CanDriver {
public:
    CanDriver();
    ~CanDriver();

    bool open(const std::string& name);
    void close();
    bool read(can_frame& frame);
    bool write(const can_frame& frame);

private:
    int sock_;    
};