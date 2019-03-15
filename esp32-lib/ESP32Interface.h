#ifndef ESP32INTERFACE_H
#define ESP32INTERFACE_H

#include "mbed.h"
#include "ESP32.h"

/*
    Interface between ESP32 driver and NetworkInterface

    Not intended to be extended
    - NetworkStack::setsockopt and NetworkStack::getsockopt not implemented
 */

// Conditions defined by 802.11 standard
#define MAX_LEN_SSID 32
#define MAX_LEN_PASSWORD 63
#define MIN_LEN_PASSWORD 8

class ESP32Interface : public NetworkStack, public WiFiInterface
{

public:

    ESP32Interface(PinName tx, PinName rx, PinName en, bool debug = false);
    virtual ~ESP32Interface(void);

    // Connectivity methods
    virtual nsapi_error_t set_credentials(const char* ssid, const char* password, nsapi_security_t security);
    virtual nsapi_error_t set_channel(uint8_t channel = 0);
    virtual nsapi_error_t connect(const char* ssid, const char* password, nsapi_security_t security = NSAPI_SECURITY_NONE, uint8_t channel = 0);
    virtual nsapi_error_t connect(void);
    virtual nsapi_error_t disconnect(void);
    virtual const char* get_ip_addr(void);
    virtual const char* get_mac_addr(void);
    virtual int8_t get_rssi(void);

    using NetworkInterface::gethostbyname;
    using NetworkInterface::add_dns_server;

    virtual nsapi_error_t scan(WiFiAccessPoint* res, unsigned int count); // override pure function

    virtual void attach_connection(Callback<void(nsapi_event_t, intptr_t)> func);
    virtual nsapi_connection_status_t get_connection_status(void) const;

protected:

    struct ESPSocket {
        int id;
        nsapi_protocol_t protocol;
        bool connected;
        SocketAddress addr;
        int keep_alive;     // Only used for TCP
    };

    // Inherited from NetworkStack
    // Socket related methods
    virtual nsapi_error_t socket_open(void** handle, nsapi_protocol_t protocol);
    virtual nsapi_error_t socket_close(void* handle);
    virtual nsapi_error_t socket_bind(void* handle, const SocketAddress& addr);
    virtual nsapi_error_t socket_listen(void* handle, int backlog);
    virtual nsapi_error_t socket_connect(void* handle, const SocketAddress& addr);
    virtual nsapi_error_t socket_accept(void* handle, void** socket, SocketAddress* addr);
    virtual nsapi_error_t socket_send(void* handle, const void* data, uint32_t size);
    virtual nsapi_error_t socket_recv(void* handle, void* buffer, uint32_t size);
    virtual nsapi_error_t socket_sendto(void* handle, const SocketAddress& addr, const void* data, uint32_t size);
    virtual nsapi_error_t socket_recvfrom(void* handle, SocketAddress* addr, void* buffer, uint32_t size);
    virtual void socket_attach(void* handle, void (*callback)(void*), void* data);

    virtual NetworkStack* get_stack(void)
    {
        return this;
    }

private:

    // AT layer
    ESP32 _esp;
    bool _initialized;

    nsapi_error_t _init(void);
    // void _hw_reset(void);           // TODO
    // void _hw_en_pulldown(void);     // TODO
    // void _hw_en_pullup(void);       // TODO
    void _update_connection_status_callback(void);

    // WiFi network credentials
    char _ssid[MAX_LEN_SSID + 1];
    char _password[MAX_LEN_PASSWORD + 1];
    nsapi_security_t _security;
    bool _credentials_valid;

    // Blocking for async connect
    bool _if_blocking;
    ConditionVariable _if_connected;

    struct _SocketInfo
    {
        bool open;
        uint16_t port;
    };
    _SocketInfo _socket_info[ESP32_SOCKET_COUNT];

    // SIGIO
    struct _Callback_SIGIO
    {
        void (*callback)(void*);
        void* data;
    };
    _Callback_SIGIO _cbsigio[ESP32_SOCKET_COUNT];
    void event(void);

    // Connection state
    nsapi_connection_status_t _connection_status;
    Callback<void(nsapi_event_t, intptr_t)> _connection_status_callback;

    nsapi_error_t _connection_status_to_error();

    // OOB processing using global event queue (shared queue from mbed_event_queue)
    EventQueue* _global_event_queue;
    // ID of events currently being processed (0 when nothing)
    int _oob_event_id;
    int _connect_event_id;
    void _process_oob_event(void);
    void _oob_to_global_event_queue(void);

    void _connect_async(void);
    Mutex _connection_mutex;
};

#endif  // ESP32INTERFACE_H
