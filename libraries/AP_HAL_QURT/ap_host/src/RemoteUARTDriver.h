#pragma once

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

/*
  Generic UART relay driver for the ap_host process.
  The DSP sends a config message containing a baudrate and a device id;
  the device id maps directly to the Linux path "/dev/ttyHS<device_id>".
*/
class RemoteUARTDriver {
public:
    RemoteUARTDriver() = default;
    ~RemoteUARTDriver();

    bool configure(uint32_t baudrate, uint32_t device_id);
    bool write(const uint8_t *buf, uint16_t len);
    bool is_open() const { return _fd != -1; }

private:
    bool _open();
    void _close();
    bool _set_speed(uint32_t baudrate);
    void _start_recv_thread();
    void _recv_loop();
    static void *_recv_thread_fn(void *arg);

    char _device_path[32] {};
    uint32_t _device_id = 0;
    int _fd = -1;

    // recv thread
    pthread_t _recv_thread_id {};
    bool _thread_running = false;
    uint32_t _rx_seq = 0;
};
