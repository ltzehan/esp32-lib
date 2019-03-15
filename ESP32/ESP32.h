#ifndef ESP32_H
#define ESP32_H

#include "mbed.h"

#define ESP32_SOCKET_COUNT 2
/*
    Default timeouts for ESP32 operations
 */
#ifndef ESP32_TIMEOUT_CONNECT
#define ESP32_TIMEOUT_CONNECT 15000
#endif  // ESP32_TIMEOUT_CONNECT

#ifndef ESP32_TIMEOUT_RECV
#define ESP32_TIMEOUT_RECV 2000
#endif  // ESP32_TIMEOUT_RECV

#ifndef ESP32_TIMEOUT_SEND
#define ESP32_TIMEOUT_SEND 2000
#endif  // ESP32_TIMEOUT_SEND

#ifndef ESP32_TIMEOUT_DEFAULT
#define ESP32_TIMEOUT_DEFAULT 2000
#endif  // ESP32_TIMEOUT_DEFAULT

/*
    ESP32 targeted firmware version
 */
#define AT_VERSION_MAJOR 1
#define AT_VERSION_MINOR 1
#define AT_VERSION_PATCH 2

// SDK version should not be a big issue
#define SDK_VERSION_MAJOR 3
#define SDK_VERSION_MINOR 0
#define SDK_VERSION_PATCH 3

/*
    ESP32 device handler
 */

class ESP32
{

public:

    ESP32(PinName tx, PinName rx, PinName en, bool debug = false);

    /**
     *  @brief ESP32 SDK version.
     *
     *  @param major Major version number
     *  @param minor Minor version number
     *  @param patch Patch version number
     */
    struct fw_sdk_version
    {
        int major;
        int minor;
        int patch;

        fw_sdk_version(int major, int minor, int patch) :
            major(major),
            minor(minor),
            patch(patch)
        {}
    };

    /**
     *  @brief ESP32 AT version.
     *
     *  @param major Major version number
     *  @param minor Minor version number
     *  @param patch Patch version number
     */
    struct fw_at_version
    {
        int major;
        int minor;
        int patch;

        fw_at_version(int major, int minor, int patch) :
            major(major),
            minor(minor),
            patch(patch)
        {}
    };

    // AT initialization methods
    bool at_ready(void);
    bool echo_off(void);

    // ESP32 hardware EN pin methods
    void hw_pulldown(void);
    void hw_pullup(void);

    //  ESP32 version querying
    fw_sdk_version get_sdk_version(void);
    fw_at_version get_at_version(void);
    bool check_fw_version(void);

    // ESP32 initialization methods
    bool configure(void);
    bool reset(void);       // Software reset (using AT)
    bool restore(void);

    //  Connectivity methods
    nsapi_error_t connect(const char* ssid, const char* password);
    bool disconnect(void);
    const char* get_ip_addr(void);
    const char* get_mac_addr(void);
    int8_t get_rssi(void);

    nsapi_connection_status_t get_connection_status(void) const;

    //  Socket related methods
    nsapi_error_t tcp_open(int id, const char* ip, int port, int keep_alive = 0);
    nsapi_error_t udp_open(int id, const char* ip, int port, int local_port = 0);
    int32_t tcp_recv(int id, void* buffer, uint32_t size, uint32_t timeout = ESP32_TIMEOUT_RECV);
    int32_t udp_recv(int id, void* buffer, uint32_t size, uint32_t timeout = ESP32_TIMEOUT_RECV);
    nsapi_error_t socket_send(int id, const void* data, uint32_t size);
    bool socket_close(int id);

    //  Serial comm. related methods
    void set_timeout(uint32_t timeout = ESP32_TIMEOUT_DEFAULT);
    void attach_connection(Callback<void()> func);
    void attach_sigio(Callback<void()> func);

    // Overload for regular callbacks
    template <typename T, typename M>
    void attach_connection(T* obj, M method)
    {
        attach_connection(Callback<void()>(obj, method));
    }
    template <typename T, typename M>
    void attach_sigio(T* obj, M method)
    {
        attach_sigio(Callback<void()>(obj, method));
    }

    void bg_process_oob(uint32_t timeout, bool all);

private:

    // ESP32 EN pin
    DigitalOut _hw_enable_pin;

    // Firmware versions
    fw_sdk_version _sdk_version;
    fw_at_version _at_version;

    // SIGIO callback
    Callback<void()> _sigio_callback;   // TODO: check if required

    // Serial settings
    Mutex _serial_mutex;
    UARTSerial _serial;
    ATCmdParser _parser;

    // Socket data buffer; data is split into multiple _Packet structs
    // which are connected like a linked list
    struct _Packet
    {
        int socket_id;
        _Packet *next;      // Pointer to next packet
        uint32_t len;       // Remaining length (read over multiple times)
        uint32_t alloc_len; // Original length of data
    };
    _Packet* _packets;
    _Packet** _packets_end;

    nsapi_error_t _socket_open(nsapi_protocol_t protocol, int id, const char* ip, int port, int param);
    int32_t _socket_recv(nsapi_protocol_t protocol, int id, void *buffer, uint32_t size, uint32_t timeout);

    struct _SocketInfo
    {
        bool open;
        nsapi_protocol_t protocol;
    };
    struct _SocketInfo _socket_info[ESP32_SOCKET_COUNT];

    static const int _SOCKET_ALL_ID = -1;
    void _clear_socket_packets(int id);

    // Memory statistics
    size_t _heap_usage;  // Socket data buffer usage

    // Out-of-band (OOB) processing
    void _process_oob(uint32_t timeout, bool all);

    // OOB handlers
    void _oob_packet_handler(void);
    void _oob_connect_status(void);
    void _oob_connect_error(void);
    void _oob_error(void);
    void _oob_socket_closed0(void);
    void _oob_socket_closed1(void);
    void _oob_busy(void);

    // OOB state variables
    int _connect_error;
    bool _connect_fail;         // If true, received +CWJAP:<error code>\nERROR response
    bool _disconnect;           // TODO: think this is true when manually disconnecting
    bool _already_connected;    // TODO: I think this is what _sock_already means
    bool _closed;
    bool _error;
    bool _busy;

    // Modem address
    char _ip_buffer[16];
    char _mac_buffer[18];

    // Connection status callback
    nsapi_connection_status_t _connection_status;
    Callback<void()> _connection_status_callback;

};

#endif  // ESP32_H
