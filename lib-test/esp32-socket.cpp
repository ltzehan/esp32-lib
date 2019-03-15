/*
    esp32-lib test for socket connection

    - Configure port numbers in socket-server.py and run
    - Configure WiFi network parameters and IP address of device running socket-server.py
 */

#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#include "ESP32.h"

#define TRACE_GROUP "MAIN"

#define ESP32_TX_PIN PC_6
#define ESP32_RX_PIN PC_7
#define ESP32_EN_PIN PC_8

#define WIFI_SSID // WIFI SSID
#define WIFI_PASSWORD // WIFI Password

#define IP_ADDR // IP Address
#define TCP_PORT 1000
#define UDP_PORT 1001

int main() {

    mbed_trace_init();
    tr_info("esp32-lib socket test");

    ESP32 esp(ESP32_TX_PIN, ESP32_RX_PIN, ESP32_EN_PIN);

    tr_info("ESP32 power cycling...");
    esp.hw_pulldown();
    ThisThread::sleep_for(1000);
    esp.hw_pullup();
    ThisThread::sleep_for(1000);

    tr_info("AT response check %s", (esp.at_ready() ? "OK" : "FAILED"));
    tr_info("AT echo off %s", (esp.echo_off() ? "OK" : "FAILED"));

    tr_info("F/W version check %s", (esp.check_fw_version() ? "OK" : "FAILED"));

    tr_info("ESP32 S/W reset %s", (esp.reset() ? "OK" : "FAILED"));

    tr_info("ESP32 configuration %s", (esp.configure() ? "OK" : "FAILED"));

    int conn = esp.connect(WIFI_SSID, WIFI_PASSWORD);
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

    int tcpsockopen = esp.tcp_open(0, IP_ADDR, TCP_PORT);
    tr_info("(TCP) Socket opening returned %d", tcpsockopen);

    if (tcpsockopen == 0)
    {
        // Send and echo
        const char tcp_data[20] = "tcp-test";
        char tcp_buffer[20] = "";

        tr_info("(TCP) Socket sending returned %d", esp.socket_send(0, tcp_data, sizeof(tcp_data)));
        tr_info("(TCP) Socket receive returned %d", esp.tcp_recv(0, tcp_buffer, sizeof(tcp_buffer)));

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

    int udpsockopen = esp.udp_open(1, IP_ADDR, UDP_PORT);
    tr_info("(UDP) Socket opening returned %d", udpsockopen);

    if (udpsockopen == 0)
    {
        // Send and echo
        const char udp_data[20] = "udp-test";
        char udp_buffer[20] = "";

        tr_info("(UDP) Socket sending returned %d", esp.socket_send(1, udp_data, sizeof(udp_data)));
        tr_info("(UDP) Socket receive returned %d", esp.udp_recv(1, udp_buffer, sizeof(udp_buffer)));

        if (strcmp(udp_data, udp_buffer) != 0)
        {
            tr_error("(UDP) Sent and received data do not match, terminating program");
        }
        else
        {
            tr_info("(UDP) Sending and receiving OK");
        }
    }

    tr_info("(TCP) Socket closing returned %d", esp.socket_close(0));
    tr_info("(UDP) Socket closing returned %d", esp.socket_close(1));

    tr_info("Disconnecting %s", (esp.disconnect() ? "OK" : "FAILED"));

    return 0;
}
