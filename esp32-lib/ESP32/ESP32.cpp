#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#include "ESP32.h"

#define TRACE_GROUP "ESP32"  // ESP32 AT layer traces

#define ESP32_DEFAULT_BAUD_RATE 115200
#define ESP32_SOCKET_BUFFERSIZE 8192

ESP32::ESP32(PinName tx, PinName rx, PinName en, bool debug) :
    _hw_enable_pin(en),
    _sdk_version(-1, -1, -1),
    _at_version(-1, -1, -1),
    _sigio_callback(0),
    _serial(tx, rx, ESP32_DEFAULT_BAUD_RATE),
    _parser(&_serial),
    _packets(0),
    _packets_end(&_packets),
    _heap_usage(0),
    _connect_error(0),
    _connect_fail(false),
    _disconnect(false),
    _already_connected(false),
    _closed(false),
    _error(false),
    _busy(false),
    _connection_status(NSAPI_STATUS_DISCONNECTED)
{
    _parser.debug_on(debug);
    _parser.set_delimiter("\r\n");

    _parser.oob("+IPD,", callback(this, &ESP32::_oob_packet_handler));
    _parser.oob("+CWJAP:", callback(this, &ESP32::_oob_connect_error)); // may not need callback
    _parser.oob("WIFI ", callback(this, &ESP32::_oob_connect_status));
    _parser.oob("ERROR", callback(this, &ESP32::_oob_error));
    _parser.oob("0,CLOSED", callback(this, &ESP32::_oob_socket_closed0));
    _parser.oob("1,CLOSED", callback(this, &ESP32::_oob_socket_closed1));
    _parser.oob("busy", callback(this, &ESP32::_oob_busy));

    for (int i = 0; i < ESP32_SOCKET_COUNT; i++)
    {
        _socket_info[i].open = false;
        _socket_info[i].protocol = NSAPI_UDP;
    }
}

/**
 *  @brief Check AT command interface of ESP32.
 *  @return If AT communication is up
 */
bool ESP32::at_ready(void)
{
    bool ready = false;
    int i = 1;

    _serial_mutex.lock();
    // Blocking; Force ESP32 AT to be up before continuing
    while (!ready)
    {
        ready = _parser.send("AT") &&
                _parser.recv("OK\n");

        tr_debug("Waiting for AT response (try %d)", i);

        ThisThread::sleep_for(ESP32_TIMEOUT_RECV);
        i++;
    }

    _serial_mutex.unlock();

    return ready;
}

/**m
 *  @brief Disable AT echoing (Required for OOB processing to work).
 *  @return If AT echo was successfully disabled
 */
bool ESP32::echo_off(void)
{
    _serial_mutex.lock();

    bool ready = _parser.send("ATE0") &&
                 _parser.recv("OK\n");

    _serial_mutex.unlock();

    return ready;
}

/**
 *  @brief Pulls ESP32 EN pin low
 */
void ESP32::hw_pulldown(void)
{
    _hw_enable_pin.write(0);
}

/**
 *  @brief Pulls ESP32 EN pin high
 */
void ESP32::hw_pullup(void)
{
    _hw_enable_pin.write(1);
}

/**
 *  @brief Get SDK version of ESP32 firmware.
 *  @return SDK version in fw_sdk_version struct
 */
ESP32::fw_sdk_version ESP32::get_sdk_version(void)
{
    int major;
    int minor;
    int patch;

    _serial_mutex.lock();

    bool done = _parser.send("AT+GMR") &&
                _parser.recv("SDK version:v%d.%d.%d", &major, &minor, &patch) &&
                _parser.recv("OK\n");

    _serial_mutex.unlock();

    if (done)
    {
        _sdk_version = fw_sdk_version(major, minor, patch);
    }

    return _sdk_version;
}

/**
 *  @brief Get AT version of ESP32 firmware
 *  @return AT version in fw_at_version struct
 */
ESP32::fw_at_version ESP32::get_at_version(void)
{
    int major;
    int minor;
    int patch;

    _serial_mutex.lock();

    bool done = _parser.send("AT+GMR") &&
                _parser.recv("AT version:%d.%d.%d", &major, &minor, &patch) &&
                _parser.recv("OK\n");

    _serial_mutex.unlock();

    if (done)
    {
        _at_version = fw_at_version(major, minor, patch);
    }

    return _at_version;
}

/**
 *  @brief Checks if ESP32 is running the targeted firmware
 *  @return True if firmware is correct
 */
bool ESP32::check_fw_version(void)
{
    fw_sdk_version sdk_ver = get_sdk_version();
    fw_at_version at_ver = get_at_version();

    bool ok = true;

    if (sdk_ver.major != SDK_VERSION_MAJOR ||
        sdk_ver.minor != SDK_VERSION_MINOR ||
        sdk_ver.patch != SDK_VERSION_PATCH)
    {
        ok = false;
        tr_error("SDK version mismatch");
    }

    if (at_ver.major != AT_VERSION_MAJOR ||
        at_ver.minor != AT_VERSION_MINOR ||
        at_ver.patch != AT_VERSION_PATCH)
    {
        ok = false;
        tr_error("AT version mismatch");
    }

    return ok;
}

/**
 *  @brief Configures ESP32 to station and single connection mode,
 *         turns on DHCP and disables autoconnection on startup.
 *  @return True if ESP32 configuration is successful
 */
bool ESP32::configure(void)
{
    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    bool done = // Set ESP32 to station mode
                _parser.send("AT+CWMODE=1") &&
                _parser.recv("OK\n") &&
                // Turn on DHCP
                _parser.send("AT+CWDHCP=1,1") &&
                _parser.recv("OK\n") &&
                // Set multiple connection mode
                _parser.send("AT+CIPMUX=1") &&
                _parser.recv("OK\n") &&
                // Disable autoconnection to AP on startup
                _parser.send("AT+CWAUTOCONN=0") &&
                _parser.recv("OK\n") &&
                // Disable IP and port address for +IPD
                _parser.send("AT+CIPDINFO=0") &&
                _parser.recv("OK\n");

    set_timeout();
    _serial_mutex.unlock();

    return done;
}

/**
 *  @brief Restarts the ESP32.
 *  @return True if successful
 */
bool ESP32::reset(void)
{
    bool done = false;

    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    // ESP32 may need a few tries to reset
    for (int i = 0; i < 2; i++)
    {
        if (_parser.send("AT+RST") &&
            _parser.recv("OK\n") &&
            _parser.recv("ready"))
        {
            done = true;
            break;
        }
    }

    _clear_socket_packets(_SOCKET_ALL_ID);

    set_timeout();
    _serial_mutex.unlock();

    return done;
}

/**
 *  @brief Restores factory settings on the ESP32.
 *  @return True if successful
 */
bool ESP32::restore(void)
{
    bool done = false;

    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    for (int i = 0; i < 2; i++) {
        if (_parser.send("AT+RESTORE")
                && _parser.recv("OK\n")
                && _parser.recv("ready")) {
            done = true;
            break;
        }
    }

    _clear_socket_packets(_SOCKET_ALL_ID);

    set_timeout();
    _serial_mutex.unlock();

    return done;
}

/**
 *  @brief Connects to a WiFi network.
 *  @param ssid SSID of network to connect to
 *  @param password Password of network to connect to
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32::connect(const char* ssid, const char* password)
{
    nsapi_error_t ret = NSAPI_ERROR_OK;

    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    _parser.send("AT+CWJAP=\"%s\",\"%s\"", ssid, password);

    if (!_parser.recv("OK\n"))
    {
        // +CWJAP:<error code> response
        if (_connect_fail)
        {
            switch (_connect_error)
            {
                case 1:
                ret = NSAPI_ERROR_CONNECTION_TIMEOUT;
                tr_error("Connection timed out");
                break;

                case 2:
                ret = NSAPI_ERROR_AUTH_FAILURE;
                tr_error("Authentication failed");
                break;

                case 3:
                ret = NSAPI_ERROR_NO_SSID;
                tr_error("SSID not found");
                break;

                case 4:
                ret = NSAPI_ERROR_NO_CONNECTION;
                tr_error("Failed to connect");
                break;
            }

            _connect_fail = false;
            _error = false;  // error OOB will set this
            _connect_error = 0;
        }
    }


    set_timeout();
    _serial_mutex.unlock();

    return ret;
}

/**
 *  @brief Disconnects from current network.
 *  @return True if successful
 */
bool ESP32::disconnect(void)
{
    _serial_mutex.lock();

    _disconnect = true;
    bool done = _parser.send("AT+CWQAP") &&
                _parser.recv("OK\n");

    _serial_mutex.unlock();

    return done;
}

/**
 *  @brief Returns IP address of ESP32 on this network.
 *  @return IP address of ESP32
 */
const char* ESP32::get_ip_addr(void)
{
    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    bool done = _parser.send("AT+CIFSR") &&
                 _parser.recv("+CIFSR:STAIP,\"%15[^\"]\"", _ip_buffer) &&
                _parser.recv("OK\n");

    set_timeout();
    _serial_mutex.unlock();

    if (!done)
    {
        return 0;
    }

    return _ip_buffer;
}

/**
 *  @brief Returns MAC address of ESP32.
 *  @return MAC address of ESP32
 */
const char* ESP32::get_mac_addr(void)
{
    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_CONNECT);

    bool done = _parser.send("AT+CIFSR") &&
                _parser.recv("+CIFSR:STAMAC,\"%17[^\"]\"", _mac_buffer) &&
                _parser.recv("OK\n");

    set_timeout();
    _serial_mutex.unlock();

    if (!done)
    {
        return 0;
    }

    return _mac_buffer;
}

/**
 *  @brief Returns RSSI of current connection.
 *  @return RSSI of current connection in dBm
 */
int8_t ESP32::get_rssi(void)
{
    // TODO Circumvent the +CWJAP response being consumed by the OOB callback
    tr_warn("RSSI not implemented yet");

    return 0;
    // int rssi;

    // // Temporarily disable +CWJAP callback or response will be consumed
    // _oob_connect_error_disabled = true;

    // _serial_mutex.lock();
    // set_timeout(ESP32_TIMEOUT_CONNECT);

    // bool done = _parser.send("AT+CWJAP?") &&
    //             _parser.recv("+CWJAP:\"%*[^\"]\",\"%*[^\"]\",%*d,%d", &rssi) &&
    //             _parser.recv("OK\n");

    // set_timeout();
    // _serial_mutex.unlock();

    // _oob_connect_error_disabled = false;

    // if (!done) {
    //     return 0;
    // }

    // return (int8_t) rssi;
}

/**
 *  @brief Returns connection status
 *  @return Connection status as nsapi_connection_status_t
 */
nsapi_connection_status_t ESP32::get_connection_status(void) const
{
    return _connection_status;
}

/**
 *  @brief Helper for opening a socket.
 *  @param id Socket id
 *  @param ip Destination IP address
 *  @param port Destination port number
 *  @param x Protocol-dependent parameter (TCP: keep alive duration; UDP: local port number)
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32::_socket_open(nsapi_protocol_t protocol, int id, const char* ip, int port, int x)
{
    const char* protocol_str = (protocol == NSAPI_TCP) ? "TCP" : "UDP";

    _serial_mutex.lock();

    _process_oob(ESP32_TIMEOUT_SEND, true);

    if (id >= ESP32_SOCKET_COUNT)
    {
        tr_error("Maximum number of sockets (%d) already open", ESP32_SOCKET_COUNT);
        _serial_mutex.unlock();

        return NSAPI_ERROR_PARAMETER;
    }

    if (_socket_info[id].open)
    {
        tr_info("Socket #%d is already open", id);
        _serial_mutex.unlock();

        return NSAPI_ERROR_PARAMETER;
    }

    bool done = false;
    for (int i = 0; i < 2; i++)
    {
        if (x == 0)
        {
            // Optional parameters not specified
            // TCP: Keep alive disabled
            // UDP: Local port number assignment is random
            done = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, protocol_str, ip, port);
        }
        else
        {
            done = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d", id, protocol_str, ip, port, x);
        }

        if (done)
        {
            // Sending AT command is OK, waiting for response
            if (!_parser.recv("OK\n"))
            {
                // No success response; check for potential errors
                if (_already_connected)
                {
                    // Socket was already open; close and retry
                    _already_connected = false;
                    done = socket_close(id);
                    if (!done)
                    {
                        // Failed to close socket; exit
                        break;
                    }
                }
                if (_error)
                {
                    _error = false;
                    done = false;
                }
            }
            else
            {
                // Successfully opened
                _socket_info[id].open = true;
                _socket_info[id].protocol = protocol;
                break;
            }
        }
    }

    _clear_socket_packets(id);

    _serial_mutex.unlock();

    if (!done)
    {
        tr_error("Failed to open %s socket #%d", protocol_str, id);
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    tr_debug("Opened %s socket #%d", protocol_str, id);
    return NSAPI_ERROR_OK;
}

/**
 *  @brief Opens a connection over TCP socket.
 *  @param id Socket id
 *  @param ip Destination IP address
 *  @param port Destination port number
 *  @param keep_alive TCP keep alive duration (0 = disabled)
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32::tcp_open(int id, const char* ip, int port, int keep_alive)
{
    tr_debug("Opening TCP socket on %s:%d", ip, port);
    return _socket_open(NSAPI_TCP, id, ip, port, keep_alive);
}

/**
 *  @brief Opens a connection over UDP socket.
 *  @param id Socket id
 *  @param ip Destination IP address
 *  @param port Destination port number
 *  @param local_port Local port number
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32::udp_open(int id, const char* ip, int port, int local_port)
{
    tr_debug("Opening UDP socket on %s:%d", ip, port);
    return _socket_open(NSAPI_UDP, id, ip, port, local_port);
}

// Helper for receiving data over TCP or UDP socket
int32_t ESP32::_socket_recv(nsapi_protocol_t protocol, int id, void *buffer, uint32_t size, uint32_t timeout)
{
    _serial_mutex.lock();
    set_timeout(timeout);

    // Not using hardware flow control; pull data out
    _process_oob(timeout, true);

    if (!_socket_info[id].open)
    {
        _serial_mutex.unlock();

        return 0;
    }

    // Check for packets
    for (_Packet** p = &_packets; *p; p = &(*p)->next)
    {
        // Check packet is for socket
        if ((*p)->socket_id == id)
        {
            _Packet* q = *p;

            uint32_t len = q->len;
            if (protocol == NSAPI_UDP && len > size)
            {
                // Truncate packet if UDP
                len = size;
            }

            if (len <= size)
            {
                // Read entire packet and remove from list
                memcpy(buffer, q+1, len);

                // Done; move on to next packet
                if (_packets_end == &(*p)->next)
                {
                    _packets_end = p;
                }
                *p = (*p)->next;

                _serial_mutex.unlock();

                // Free memory for this packet
                uint32_t p_len = sizeof(_Packet) + q->alloc_len;
                free(q);
                _heap_usage -= p_len;

                return len;
            }
            else
            {
                // Read only part of packet
                memcpy(buffer, q+1, size);

                // Shift unread data to front of packet
                q->len -= size;
                memmove(q+1, (uint8_t*)(q+1) + size, q->len);

                _serial_mutex.unlock();

                return size;
            }
        }
    }

    _serial_mutex.unlock();

    return NSAPI_ERROR_WOULD_BLOCK;
}

/**
 *  @brief Receives data from a TCP socket.
 *  @param id Socket id
 *  @param buffer Buffer for receiving data
 *  @param size Number of bytes of data to be received
 *  @param timeout Timeout for receiving data
 *  @return Number of bytes received (negative indicates error)
 */
int32_t ESP32::tcp_recv(int id, void* buffer, uint32_t size, uint32_t timeout)
{
    return _socket_recv(NSAPI_TCP, id, buffer, size, timeout);
}

/**
 *  @brief Receives data from a UDP socket.
 *  @param id Socket id
 *  @param buffer Buffer for receiving data
 *  @param size Number of bytes of data to be received
 *  @param timeout Timeout for receiving data
 *  @return Number of bytes received (negative indicates error)
 */
int32_t ESP32::udp_recv(int id, void* buffer, uint32_t size, uint32_t timeout)
{
    return _socket_recv(NSAPI_UDP, id, buffer, size, timeout);
}

/**
 *  @brief Sends data through a socket.
 *  @param id Socket id
 *  @param data Data to be sent
 *  @param size Number of bytes of data to send
 *  @return nsapi_error_t return code
 */
nsapi_error_t ESP32::socket_send(int id, const void* data, uint32_t size)
{
    nsapi_error_t ret = NSAPI_ERROR_DEVICE_ERROR;

    // Only 2048 bytes can be sent at a time using +CIPSEND
    if (size > 2048)
    {
        tr_warn("Attempting to send >2048 bytes at once");

        if (_socket_info[id].protocol == NSAPI_TCP)
        {
            size = 2048;
        }
        else if (_socket_info[id].protocol == NSAPI_UDP)
        {
            tr_error("Cannot send >2048 bytes over UDP");
            return NSAPI_ERROR_PARAMETER;
        }
    }

    _serial_mutex.lock();
    set_timeout(ESP32_TIMEOUT_SEND);

    _busy = false;
    _error = false;

    if (_parser.send("AT+CIPSEND=%d,%lu", id, size))
    {
        if (_parser.recv(">"))
        {
            // ESP32 ready to receive data
            if (_parser.write((char*) data, (int) size) >= 0 &&
                _parser.recv("SEND OK"))
            {
                // Data sending was successful
                ret = NSAPI_ERROR_OK;
            }
            else
            {
                tr_warn("Data sending was not complete");
            }
        }
        else
        {
            // Timed out; device did not respond
            tr_warn("Did not receive \">\" after AT+CIPSEND");
            ret = NSAPI_ERROR_WOULD_BLOCK;
        }
    }
    else
    {
        tr_warn("AT+CIPSEND failed");
    }

    // Drain UART receive register
    _process_oob(ESP32_TIMEOUT_RECV, true);

    if (_busy)
    {
        ret = NSAPI_ERROR_WOULD_BLOCK;
        tr_warn("ESP32 busy");
    }

    if (_error) {
        ret = NSAPI_ERROR_CONNECTION_LOST;
        tr_warn("ESP32 connection disrupted");
    }

    if (!_socket_info[id].open && ret != NSAPI_ERROR_OK) {
        ret = NSAPI_ERROR_CONNECTION_LOST;
        tr_warn("Socket #%d closed abruptly", id);
    }

    set_timeout();
    _serial_mutex.unlock();

    return ret;
}

/**
 *  @brief Closes a socket.
 *  @param id Socket id
 *  @return True if successful
 */
bool ESP32::socket_close(int id)
{
    _serial_mutex.lock();

    bool done = false;
    for (int i = 0; i < 2; i++)
    {
        if (_parser.send("AT+CIPCLOSE=%d", id))
        {
            if (_parser.recv("OK\n"))
            {
                // _socket_info[id].open should be set to false by OOB
                _clear_socket_packets(id);
                done = true;
                break;
            }
            else
            {
                if (_closed)
                {
                    // Socket closed on its own
                    _closed = false;
                    _socket_info[id].open = false;
                    _clear_socket_packets(id);
                    done = true;
                    break;
                }
            }
        }
    }

    _serial_mutex.unlock();

    if (!done)
    {
        tr_warn("Failed to close socket #%d", id);
        return false;
    }

    return true;
}

/**
 *  @brief Clears all packets in socket buffer.
 *  @param id Socket id
 */
void ESP32::_clear_socket_packets(int id)
{
    _Packet** p = &_packets;

    while (*p)
    {
        if ((*p)->socket_id == id || id == _SOCKET_ALL_ID)
        {
            _Packet* q = *p;
            uint32_t p_len = sizeof(_Packet) + q->alloc_len;

            // Free packets from head of _packets (linked list)
            if (_packets_end == &(*p)->next)
            {
                _packets_end = p;
            }
            *p = (*p)->next;
            free(q);
            _heap_usage -= p_len;
        }
        else
        {
            // Move on to next packet
            p = &(*p)->next;
        }
    }
}

/**
 *  @brief Sets timeout for AT command R/W.
 *  @param timeout Timeout in ms
 */
void ESP32::set_timeout(uint32_t timeout)
{
    _parser.set_timeout(timeout);
}

/**
 *  @brief Attaches callback for network connectivity change.
 *  @param func Callback method
 */
void ESP32::attach_connection(Callback<void()> func)
{
    _connection_status_callback = func;
}

/**
 *  @brief Attaches callback for SIGIO signal.
 *  @param func Callback method
 */
void ESP32::attach_sigio(Callback<void()> func)
{
    _serial.sigio(func);
    _sigio_callback = func;
}

////////////////////////////////////////////////////
//
//      Out-of-band callbacks and handlers
//
////////////////////////////////////////////////////

/**
 *  @brief Processes OOBs.
 *  @param timeout AT command timeout
 *  @param If true, process all OOBs instead of only one
 */
void ESP32::_process_oob(uint32_t timeout, bool all)
{
    set_timeout(timeout);

    // Poll for inbound OOBs
    // ATCmdParser::process_oob returns false when nothing left to process
    while (_parser.process_oob() && all)
    {}

    set_timeout();
}

/**
 *  @brief Executes OOB processing in the background.
 *         Wraps ESP32::_process_oob in Serial mutex.
 *  @param timeout AT command timeout
 *  @param If true, process all OOBs instead of only one
 */
void ESP32::bg_process_oob(uint32_t timeout, bool all)
{
    _serial_mutex.lock();
    _process_oob(timeout, all);
    _serial_mutex.unlock();
}

/**
 *  @brief Handles "+IPD" OOB.
 *         +IPD,<id>,<len>:<data>
 */
void ESP32::_oob_packet_handler(void)
{
    int id;
    int size;

    if (!_parser.recv("%d,%d:", &id, &size))
    {
        // +IPD does not match expected syntax
        tr_warn("ESP32 +IPD response has unexpected syntax");

        return;
    }

    int p_len = sizeof(_Packet) + size;

    if ((_heap_usage + p_len) > ESP32_SOCKET_BUFFERSIZE)
    {
        // Data will overflow socket buffer
        tr_warn("ESP32 socket buffer is full; Packet dropped");

        return;
    }

    _Packet* packet = (_Packet*) malloc(p_len);
    if (!packet)
    {
        // Pointer is NULL, cannot allocate memory
        tr_warn("Out of memory, unable to allocate memory for packet");

        return;
    }
    _heap_usage += p_len;

    packet->socket_id = id;
    packet->next = 0;
    packet->len = size;
    packet->alloc_len = size;

    if (_parser.read((char*)(packet+1), size) < size)
    {
        // Packet is shorter than expected; malformed
        free(packet);
        _heap_usage -= p_len;

        return;
    }

    // Append to packet linked list
    *_packets_end = packet;
    _packets_end = &packet->next;

}

/**
 *  @brief Callback for "WIFI " OOB.
 *         WIFI GOT IP
 *         WIFI CONNECTED
 *         WIFI DISCONNECT
 */
void ESP32::_oob_connect_status(void)
{
    char status[13];
    if (_parser.recv("%12[^\"]\n", status))
    {
        if (strcmp(status, "GOT IP\n") == 0)
        {
            _connection_status = NSAPI_STATUS_GLOBAL_UP;
        }
        else if (strcmp(status, "CONNECTED\n") == 0)
        {
            _connection_status = NSAPI_STATUS_CONNECTING;
        }
        else if (strcmp(status, "DISCONNECT\n") == 0)
        {
            if (_disconnect)
            {
                // Disconnection was by user
                _connection_status = NSAPI_STATUS_DISCONNECTED;
                _disconnect = false;
            }
            else
            {
                _connection_status = NSAPI_STATUS_CONNECTING;
            }
        }
        else
        {
            tr_warn("Unrecognized WiFi status \"%s\"", status);
        }
    }
    else
    {
        // Timed out
        tr_info("WiFi connection timed out");
    }

    if (_connection_status_callback)
    {
        _connection_status_callback();
    }
}

/**
 *  @brief Callback for "+CWJAP:" OOB.
 *         +CWJAP:<error code>\nERROR
 */
void ESP32::_oob_connect_error(void)
{
    _connect_fail = false;
    _connect_error = 0;

    // TODO this may raise error OOB
    if (_parser.recv("%d", &_connect_error) && _parser.recv("ERROR"))
    {
        _connect_fail = true;
        _error = false;  // Already handled by _connect_fail
        _parser.abort();
    }
}

/**
 *  @brief Callback for "ERROR" OOB.
 */
void ESP32::_oob_error(void)
{
    _error = true;
    _parser.abort();
}

/**
 *  @brief Callback for "0,CLOSED" OOB.
 */
void ESP32::_oob_socket_closed0(void)
{
    _socket_info[0].open = false;
    tr_debug("Socket #0 closed");
}

/**
 *  @brief Callback for "1,CLOSED" OOB.
 */
void ESP32::_oob_socket_closed1(void)
{
    _socket_info[1].open = false;
    tr_debug("Socket #1 closed");
}

/**
 *  @brief Callback for "busy" OOB.
 *         busy p...
 */
void ESP32::_oob_busy(void)
{
    // ESP32 only has busy p... state unlike ESP8266 which also has busy s...
    _busy = true;
    _parser.abort();
}
