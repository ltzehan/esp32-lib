#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#include "ESP32Interface.h"

#define TRACE_GROUP "ESPI"  // ESP32Interface traces

ESP32Interface::ESP32Interface(PinName tx, PinName rx, PinName en, bool debug) :
    _esp(tx, rx, en, debug),
    _initialized(false),
    _security(NSAPI_SECURITY_UNKNOWN),
    _credentials_valid(false),
    _if_blocking(true),
    _if_connected(_connection_mutex),
    _connection_status(NSAPI_STATUS_DISCONNECTED),
    _connection_status_callback(0),
    _global_event_queue(0),
    _oob_event_id(0),
    _connect_event_id(0)
{
    memset(_socket_info, 0, sizeof(_socket_info));
    memset(_cbsigio, 0, sizeof(_cbsigio));

    _esp.set_timeout();
    _esp.attach_sigio(this, &ESP32Interface::event);
    _esp.attach_connection(this, &ESP32Interface::_update_connection_status_callback);

    for (int id = 0; id < ESP32_SOCKET_COUNT; id++)
    {
        _socket_info[id].open = false;
        _socket_info[id].port = 0;
    }

    _oob_to_global_event_queue();
}

ESP32Interface::~ESP32Interface(void)
{
    // Cancel events that are currently processing

    if (_oob_event_id != 0)
    {
        _global_event_queue->cancel(_oob_event_id);
    }

    _connection_mutex.lock();
    if (_connect_event_id != 0)
    {
        _global_event_queue->cancel(_connect_event_id);
    }
    _connection_mutex.unlock();

    // TODO _hw_en_pulldown();
}

/**
 *  @brief Checks validity and sets network connection configuration.
 *  @param ssid SSID of network
 *  @param password Password of network
 *  @param security Security type of network in nsapi_security_t type
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::set_credentials(const char* ssid, const char* password, nsapi_security_t security)
{
    _credentials_valid = false;

    if (!ssid)
    {
        tr_warn("No SSID provided");

        return NSAPI_ERROR_NO_SSID;
    }

    int ssid_len = strlen(ssid);
    if (ssid_len > 0 && ssid_len <= MAX_LEN_SSID)
    {
        memset(_ssid, 0, sizeof(_ssid));
        strncpy(_ssid, ssid, sizeof(_ssid));
    }
    else
    {
        // Invalid SSID
        tr_warn("SSID provided is invalid");

        return NSAPI_ERROR_PARAMETER;
    }

    if (security != NSAPI_SECURITY_NONE)
    {
        // Require valid password
        if (!password)
        {
            tr_warn("No password provided, is WiFi security misconfigured?");

            return NSAPI_ERROR_PARAMETER;
        }

        int password_len = strlen(password);
        if (password_len >= MIN_LEN_PASSWORD && password_len <= MAX_LEN_PASSWORD)
        {
            memset(_password, 0, sizeof(_password));
            memcpy(_password, password, sizeof(_password));
        }
        else
        {
            return NSAPI_ERROR_PARAMETER;
        }
    }

    _credentials_valid = true;

    return NSAPI_ERROR_OK;
}

/**
 *  @brief [NOT SUPPORTED] From extending WiFiInterface.
 *  @param channel [NOT SUPPORTED] Should always be 0
 *  @return Will always return NSAPI_ERROR_UNSUPPORTED
 */
nsapi_error_t ESP32Interface::set_channel(uint8_t channel)
{
    tr_warn("ESP32 channel configuration is not supported");

    return NSAPI_ERROR_UNSUPPORTED;
}

/**
 *  @brief Connects to a WiFi network and also initializes ESP32 if needed.
 *  @param ssid SSID of network
 *  @param password Password of network
 *  @param security Security type of network in nsapi_security_t type
 *  @param channel [NOT SUPPORTED] Should always be 0
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::connect(const char* ssid, const char* password, nsapi_security_t security, uint8_t channel)
{
    if (channel != 0)
    {
        tr_warn("ESP32Interface::connect should be called with channel = 0");

        return NSAPI_ERROR_UNSUPPORTED;
    }

    nsapi_error_t rc = set_credentials(ssid, password, security);
    if (rc != NSAPI_ERROR_OK)
    {
        return rc;
    }

    return connect();
}

/**
 *  @brief Connects to a WiFi network and also initializes ESP32 if needed.
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::connect(void)
{
    if (!_credentials_valid)
    {
        tr_warn("Attempted to connect with invalid credentials");

        return NSAPI_ERROR_PARAMETER;
    }

    nsapi_error_t status = _connection_status_to_error();
    if (status != NSAPI_ERROR_NO_CONNECTION)
    {
        // Device should be in disconnected state
        // Either already connected or has error
        tr_warn("Connection status invalid");

        return status;
    }

    // Initialize ESP32
    status = _init();
    if (status != NSAPI_ERROR_OK)
    {
        // Failed to initialize ESP32
        tr_error("Failed to initialize ESP32");

        return status;
    }

    if (get_ip_addr())
    {
        // If result is not NULL, device is already connected
        return NSAPI_ERROR_IS_CONNECTED;
    }

    _connection_mutex.lock();

    _connect_event_id = _global_event_queue->call(callback(this, &ESP32Interface::_connect_async));
    if (_connect_event_id == 0)
    {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                        "Failed to add connect event to queue");
    }

    // Blocking connect just waits for _connect_async to return
    while (_if_blocking && (_connection_status_to_error() != NSAPI_ERROR_IS_CONNECTED))
    {
        _if_connected.wait();
    }

    // Delay after _connect_async releases mutex
    ThisThread::sleep_for(10);

    _connection_mutex.unlock();

    return NSAPI_ERROR_OK;
}

/**
 *  @brief Asynchronously connects to network
 */
void ESP32Interface::_connect_async(void)
{
    _connection_mutex.lock();

    if (_connect_event_id == 0)
    {
        tr_debug("Cancelled _connect_async event");
        _connection_mutex.unlock();

        return;
    }

    if (_esp.connect(_ssid, _password) != NSAPI_ERROR_OK)
    {
        // Queue another call for _connect_async
        _connect_event_id = _global_event_queue->call_in(ESP32_TIMEOUT_CONNECT, callback(this, &ESP32Interface::_connect_async));
        if (_connect_event_id == 0)
        {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                            "Failed to add connect event to queue");
        }
    }
    else
    {
        _connect_event_id = 0;
        _if_connected.notify_all();
    }

    _connection_mutex.unlock();
}

/**
 *  @brief Disconnects from WiFi network
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::disconnect(void)
{
    _connection_mutex.lock();

    if (_connect_event_id != 0)
    {
        // Cancel event
        _global_event_queue->cancel(_connect_event_id);
        _connect_event_id = 0;
    }

    _connection_mutex.unlock();

    _initialized = false;   // TODO: should this be after disconnect attempt

    nsapi_error_t status = _connection_status_to_error();
    if (status == NSAPI_ERROR_NO_CONNECTION || !get_ip_addr())
    {
        // ESP32 is not connected in the first place
        return NSAPI_ERROR_NO_CONNECTION;
    }

    nsapi_error_t rc = NSAPI_ERROR_DEVICE_ERROR;
    if (_esp.disconnect())
    {
        rc = NSAPI_ERROR_OK;

        // Wait for new status update
        _esp.bg_process_oob(ESP32_TIMEOUT_RECV, true);
        // If still not updated, attach to callback
        if (_connection_status != NSAPI_STATUS_DISCONNECTED)
        {
            _connection_status = NSAPI_STATUS_DISCONNECTED;
            if (_connection_status_callback)
            {
                _connection_status_callback(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _connection_status);
            }
        }
    }

    // TODO _hw_en_pulldown();

    return rc;
}

/**
 *  @brief Gets the ESP32 IP address
 *  @return IP address or NULL if not connected
 */
const char* ESP32Interface::get_ip_addr(void)
{
    const char* ip_buffer = _esp.get_ip_addr();
    if (!ip_buffer || strcmp(ip_buffer, "0.0.0.0") == 0)
    {
        return NULL;
    }

    return ip_buffer;
}

/**
 *  @brief Gets the ESP32 MAC address
 *  @return MAC address
 */
const char* ESP32Interface::get_mac_addr(void)
{
    return _esp.get_mac_addr();
}

/**
 *  @brief Gets RSSI of network
 *  @return RSSI in dBm
 */
int8_t ESP32Interface::get_rssi(void)
{
    return _esp.get_rssi();
}

/**
 *  @brief [NOT SUPPORTED] From extending WiFiInterface.
 *  @param res Pointer to store discovered APs
 *  @param count Size of res
 *  @return Number of APs if successful or negative nsapi_error_t
 */
nsapi_error_t ESP32Interface::scan(WiFiAccessPoint* res, unsigned int count)
{
    return NSAPI_STATUS_ERROR_UNSUPPORTED;
}

/**
 *  @brief Sets callback for connection status change
 *  @param func Callback
 */
void ESP32Interface::attach(Callback<void(nsapi_event_t, intptr_t)> func)
{
    _connection_status_callback = func;
}

/**
 *  @return Connection status
 */
nsapi_connection_status_t ESP32Interface::get_connection_status(void) const
{
    return _connection_status;
}

/**
 *  @brief Convert connection status to nsapi_error_t
 *  @return Conncetion status as nsapi_error_t type
 */
nsapi_error_t ESP32Interface::_connection_status_to_error(void)
{
    nsapi_error_t rc;

    _esp.bg_process_oob(ESP32_TIMEOUT_RECV, true);

    switch (_connection_status) {
        case NSAPI_STATUS_DISCONNECTED:
            rc = NSAPI_ERROR_NO_CONNECTION;
            break;
        case NSAPI_STATUS_CONNECTING:
            rc = NSAPI_ERROR_ALREADY;
            break;
        case NSAPI_STATUS_GLOBAL_UP:
            rc = NSAPI_ERROR_IS_CONNECTED;
            break;
        default:
            rc = NSAPI_ERROR_DEVICE_ERROR;
    }

    return rc;
}

void ESP32Interface::_update_connection_status_callback(void)
{
    nsapi_connection_status_t prev = _connection_status;
    _connection_status = _esp.get_connection_status();

    if (prev == _connection_status)
    {
        return;
    }

    switch (_connection_status) {
        // No change
        case NSAPI_STATUS_CONNECTING:
        case NSAPI_STATUS_GLOBAL_UP:
            break;
        // Restart
        case NSAPI_STATUS_DISCONNECTED:
            break;
        // AT layer level
        case NSAPI_STATUS_LOCAL_UP:
        case NSAPI_STATUS_ERROR_UNSUPPORTED:
        default:
            _initialized = false;
            _connection_status = NSAPI_STATUS_DISCONNECTED;
    }

    if (_connection_status_callback)
    {
        _connection_status_callback(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _connection_status);
    }

}

/**
 *  @brief Opens a socket.
 *  @param handle Handle to store new Socket
 *  @param protocol Socket protocol to be opened
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_open(void** handle, nsapi_protocol_t protocol)
{
    // Look for unused socket
    int id = - 1;
    for (int i = 0; i < ESP32_SOCKET_COUNT; i++)
    {
        if (!_socket_info[i].open)
        {
            id = i;
            _socket_info[i].open = true;
            break;
        }
    }

    if (id == -1)
    {
        // Failed to find available socket
        tr_warn("No sockets available");

        return NSAPI_ERROR_NO_SOCKET;
    }

    ESPSocket* socket = new ESPSocket;
    if (!socket)
    {
        tr_error("Cannot allocate memory for socket");

        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = id;
    socket->protocol = protocol;
    socket->connected = false;
    socket->keep_alive = 0;

    *handle = socket;

    return NSAPI_ERROR_OK;
}

/**
 *  @brief Close the socket.
 *  @param handle Socket handle
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_close(void* handle)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    int rc = NSAPI_ERROR_OK;
    if (socket->connected && !_esp.socket_close(socket->id))
    {
        rc = NSAPI_ERROR_DEVICE_ERROR;
    }

    socket->connected = false;
    _socket_info[socket->id].open = false;
    _socket_info[socket->id].port = 0;

    delete socket;

    return rc;
}

/**
 *  @brief Bind server socket to specific port.
 *  @param handle Socket handle
 *  @param addr Local address
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_bind(void* handle, const SocketAddress& addr)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->protocol == NSAPI_UDP)
    {
        if (addr.get_addr().version != NSAPI_UNSPEC)
        {
            // Should be a local address
            return NSAPI_ERROR_UNSUPPORTED;
        }

        for (int id = 0; id < ESP32_SOCKET_COUNT; id++)
        {
            if (_socket_info[id].port == addr.get_port() && id != socket->id)
            {
                // Socket is on same port but has different id
                return NSAPI_ERROR_PARAMETER;
            }
            else if (id == socket->id && socket->connected)
            {
                // Socket is already connected
                return NSAPI_ERROR_PARAMETER;
            }
        }

        _socket_info[socket->id].port = addr.get_port();

        return NSAPI_ERROR_OK;
    }

    // Not allowed for TCP
    return NSAPI_ERROR_UNSUPPORTED;
}

/**
 *  @brief [NOT SUPPORTED] From extending NetworkStack.
 *  @param handle Socket handle
 *  @param backlog [NOT SUPPORTED]
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_listen(void* handle, int backlog)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

/**
 *  @brief Connects TCP socket.
 *  @param handle Socket handle
 *  @param addr Local address
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_connect(void* handle, const SocketAddress& addr)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    nsapi_error_t rc;
    if (socket->protocol == NSAPI_UDP)
    {
        rc = _esp.udp_open(socket->id, addr.get_ip_address(), addr.get_port(), _socket_info[socket->id].port);
    }
    else if (socket->protocol == NSAPI_TCP)
    {
        rc = _esp.tcp_open(socket->id, addr.get_ip_address(), addr.get_port(), socket->keep_alive);
    }

    socket->connected = (rc == NSAPI_ERROR_OK);

    return rc;
}

/**
 *  @brief [NOT SUPPORTED] From extending NetworkStack.
 *  @param handle Socket handle
 *  @param socket Socket handle to server to accept from
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_accept(void* handle, void** socket, SocketAddress* addr)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

/**
 *  @brief Send data.
 *  @param handle Socket handle
 *  @param data Buffer for data to be sent
 *  @param size Number of bytes to send
 *  @return Number of bytes sent if successful,
 *          negative corresponds to nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_send(void* handle, const void* data, uint32_t size)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!size)
    {
        // Size must be specified; limitation of ESP32 firmware
        return NSAPI_ERROR_UNSUPPORTED;
    }

    // Try sending for 50ms
    nsapi_error_t status;
    unsigned long int start_time = rtos::Kernel::get_ms_count();
    do
    {
        status = _esp.socket_send(socket->id, data, size);
    }
    while ((start_time - rtos::Kernel::get_ms_count() < 50) && (status != NSAPI_ERROR_OK));

    if (status == NSAPI_ERROR_WOULD_BLOCK)
    {
        tr_debug("Queueing event()");
        _global_event_queue->call_in(100, callback(this, &ESP32Interface::event));
    }

    if (status != NSAPI_ERROR_OK)
    {
        return status;
    }

    return size;
}

/**
 *  @brief Receive data.
 *  @param handle Socket handle
 *  @param buffer Buffer to store received data
 *  @param size Maximum size of buffer
 *  @return Number of bytes received if successful,
 *          negative corresponds to nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_recv(void* handle, void* buffer, uint32_t size)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    int32_t received;
    if (socket->protocol == NSAPI_TCP)
    {
        received = _esp.tcp_recv(socket->id, buffer, size);
        if (received <= 0 && received != NSAPI_ERROR_WOULD_BLOCK)
        {
            socket->connected = false;
        }
    }
    else
    {
        received = _esp.udp_recv(socket->id, buffer, size);
    }

    return received;
}

/**
 *  @brief Send a packet.
 *  @param handle Socket handle
 *  @param addr Remote address
 *  @param data Packet to be sent
 *  @param size Number of bytes of packet
 *  @return Number of bytes sent if successful,
 *          negative corresponds to nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_sendto(void* handle, const SocketAddress& addr, const void* data, uint32_t size)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if ((strcmp(addr.get_ip_address(), "0.0.0.0") == 0) || !addr.get_port())  {
        return NSAPI_ERROR_DNS_FAILURE;
    }

    if (socket->connected && socket->addr != addr)
    {
        // Wrong address
        if (!_esp.socket_close(socket->id))
        {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        socket->connected = false;
    }

    if (!socket->connected)
    {
        int rc = socket_connect(socket, addr);
        if (rc < 0)
        {
            // Returned nsapi_error_t
            return rc;
        }
        socket->addr = addr;
    }

    return socket_send(socket, data, size);
}

/**
 *  @brief Receive a packet.
 *  @param handle Socket handle
 *  @param addr Destination address
 *  @param buffer Buffer for storing received data
                  If packet is too long, excess bytes will be discarded
 *  @param size Size of buffer
 *  @return Number of bytes received if successful,
 *          negative corresponds to nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::socket_recvfrom(void* handle, SocketAddress* addr, void* buffer, uint32_t size)
{
    ESPSocket* socket = (ESPSocket*) handle;
    if (!socket)
    {
        return NSAPI_ERROR_NO_SOCKET;
    }

    nsapi_error_t rc = socket_recv(socket, buffer, size);
    if (rc >= 0 && addr)
    {
        *addr = socket->addr;
    }

    return rc;
}

/**
 *  @brief Attach callback for socket state change.
 *  @param handle Socket handle
 *  @param callback Callback
 *  @param data Argument to pass to callback
 */
void ESP32Interface::socket_attach(void* handle, void (*callback)(void*), void* data)
{
    ESPSocket* socket = (ESPSocket*) handle;

    _cbsigio[socket->id].callback = callback;
    _cbsigio[socket->id].data = data;
}

/**
 *  @brief Initializes ESP32
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32Interface::_init(void)
{
    if (!_initialized)
    {
        // Power cycle the ESP32
        _esp.hw_pulldown();
        ThisThread::sleep_for(1000);
        _esp.hw_pullup();
        ThisThread::sleep_for(1000);

        if (!_esp.at_ready() || !_esp.echo_off())
        {
            tr_error("AT setup failed");

            return NSAPI_ERROR_DEVICE_ERROR;
        }

        if (!_esp.check_fw_version())
        {
            tr_warn("Firmware is not the targeted version");

            return NSAPI_ERROR_DEVICE_ERROR;
        }

        if (!_esp.configure())
        {
            tr_warn("Failed to configure ESP32");

            return NSAPI_ERROR_DEVICE_ERROR;
        }

        _initialized = true;
    }

    return NSAPI_ERROR_OK;
}

/**
 *  @brief Process SIGIO callbacks
 */
void ESP32Interface::event(void)
{
    for (int id = 0; id < ESP32_SOCKET_COUNT; id++)
    {
        if (_cbsigio[id].callback)
        {
            _cbsigio[id].callback(_cbsigio[id].data);
        }
    }
}

void ESP32Interface::_process_oob_event(void)
{
    _esp.bg_process_oob(ESP32_TIMEOUT_RECV, true);
}

/**
 *  @brief Periodically adds OOB callback to event queue
 */
void ESP32Interface::_oob_to_global_event_queue(void)
{
    _global_event_queue = mbed_event_queue();
    _oob_event_id = _global_event_queue->call_every(ESP32_TIMEOUT_RECV, callback(this, &ESP32Interface::_process_oob_event));

    if (!_oob_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                   "Unable to queue OOB event on global event queue");
    }
}
