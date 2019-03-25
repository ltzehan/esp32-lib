/*
    esp32-lib test for ESP32Interface

    - Configure port numbers in socket-server.py and run
    - Configure WiFi network parameters and IP address of device running socket-server.py
    - Set the Root CA certificate (for the certificate in socket-server.py)
 */

#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#include "TCPSocket.h"
#include "ESP32Interface.h"

#define TRACE_GROUP "MAIN"

#define ESP32_TX_PIN PC_6
#define ESP32_RX_PIN PC_7
#define ESP32_EN_PIN PC_8

#define WIFI_SSID  // WiFi SSID
#define WIFI_PASSWORD  // WiFi Password

#define IP_ADDR  // IP Address
#define TCP_PORT 1000
#define UDP_PORT 1001
#define TLS_PORT 1002

const char ROOT_CA_CERT[] = "";  // Root CA certificate
const char CLIENT_CERT[] = "";  // Client certificate
const char CLIENT_KEY[] = "";  // Client private key

int main() {

    mbed_trace_init();
    tr_info("esp32-lib ESP32Interface test");

    /*
     *  ESP32 & WiFi connection
     */
    ESP32Interface esp(ESP32_TX_PIN, ESP32_RX_PIN, ESP32_EN_PIN);

    int conn = esp.connect(WIFI_SSID, WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);

    tr_info("Connection returned %d", conn);
    if (conn == 0)
    {
        tr_info("IP = %s", esp.get_ip_addr());
        tr_info("MAC = %s", esp.get_mac_addr());
//        tr_info("RSSI = %d", esp.get_rssi());
    }
    else
    {
        tr_error("Failed to connect, terminating program");
        return -1;
    }

    /*
     *  TCP Socket
     */
    TCPSocket tcpsock;

    int tcpsockopen = tcpsock.open(&esp);
    tr_info("(TCP) Socket opening returned %d", tcpsockopen);

    int tcpsockconn = tcpsock.connect(IP_ADDR, TCP_PORT);
    tr_info("(TCP) Socket connection returned %d", tcpsockconn);

    if (tcpsockopen == 0 && tcpsockconn == 0)
    {
        // Send and echo
        const char tcp_data[20] = "tcp-test";
        char tcp_buffer[20] = "";

        tr_info("(TCP) Socket sending returned %d", tcpsock.send(tcp_data, sizeof(tcp_data)));
        tr_info("(TCP) Socket receive returned %d", tcpsock.recv(tcp_buffer, sizeof(tcp_buffer)));

        if (strcmp(tcp_data, tcp_buffer) != 0)
        {
            tr_error("(TCP) Sent and received data do not match, terminating program");
            return -1;
        }
        else
        {
            tr_info("(TCP) Sending and receiving OK");
        }
    }

    ThisThread::sleep_for(1000);

    /*
     *  UDP Socket
     */
    UDPSocket udpsock;

    int udpsockopen = udpsock.open(&esp);
    tr_info("(UDP) Socket opening returned %d", udpsockopen);

    if (udpsockopen == 0)
    {
        // Send and echo
        const char udp_data[20] = "udp-test";
        char udp_buffer[20] = "";

        tr_info("(UDP) Socket sending returned %d", udpsock.sendto(IP_ADDR, UDP_PORT, udp_data, sizeof(udp_data)));
        tr_info("(UDP) Socket receive returned %d", udpsock.recvfrom(NULL, udp_buffer, sizeof(udp_buffer)));

        if (strcmp(udp_data, udp_buffer) != 0)
        {
            tr_error("(UDP) Sent and received data do not match, terminating program");
            return -1;
        }
        else
        {
            tr_info("(UDP) Sending and receiving OK");
        }
    }

    ThisThread::sleep_for(1000);

    /*
     *  TLS Socket
     */
    TLSSocket* tlssock = new TLSSocket();

    int tlssockca = tlssock->set_root_ca_cert(CERT);
    tr_info("(TLS) Setting Root CA returned %d", tlssockca);

    int tlssockcli = tlssock->set_client_cert_key(CLIENT_CERT, CLIENT_KEY);
    tr_info("(TLS) Setting client cert returned %d", tlssockcli);

    int tlssockopen = tlssock->open(&esp);
    tr_info("(TLS) Socket opening returned %d", tlssockopen);

    int tlssockconn = tlssock->connect(IP_ADDR, TLS_PORT);
    tr_info("(TLS) Socket connection returned %d", tlssockconn);

    if (tlssockopen == 0 && tlssockconn == 0)
    {
        // Send and echo
        const char tls_data[20] = "tls-test";
        char tls_buffer[20] = "";

        tr_info("(TLS) Socket sending returned %d", tlssock->send(tls_data, sizeof(tls_data)));
        tr_info("(TLS) Socket receive returned %d", tlssock->recv(tls_buffer, sizeof(tls_buffer)));

        if (strcmp(tls_data, tls_buffer) != 0)
        {
            tr_error("(TLS) Sent and received data do not match, terminating program");
            return -1;
        }
        else
        {
            tr_info("(TLS) Sending and receiving OK");
        }
    }

    tr_info("(TCP) Socket closing returned %d", tcpsock.close());
    tr_info("(UDP) Socket closing returned %d", udpsock.close());
    tr_info("(TLS) Socket closing returned %d", tlssock->close());

    tr_info("Disconnecting returned %d", esp.disconnect());

    return 0;
}
