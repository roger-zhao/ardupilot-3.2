#ifndef __AP_HAL_LINUX_PTSDEVICE_H__
#define __AP_HAL_LINUX_PTSDEVICE_H__

#include <stdint.h>
#include <stdlib.h>
//#include <AP_HAL/utility/Socket.h>

class PTSDevice {
public:
    PTSDevice() {};
    PTSDevice(const char *device_path);
    PTSDevice(int _fd_pts);
    virtual ~PTSDevice();

    bool open();
    bool close();
    ssize_t write(const uint8_t *buf, uint16_t n);
    ssize_t read(uint8_t *buf, uint16_t n);
    void set_blocking(bool blocking);
    void set_speed(uint32_t speed);

private:
    void _disable_crlf();

    int _fd = -1;
    const char* _device_path;
#define MAX_PTS_PATH 128
    char pts_slave_path[MAX_PTS_PATH];
};

#endif
