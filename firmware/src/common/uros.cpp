#include "uros.h"

#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <rmw_microros/rmw_microros.h>

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#error "Copy firmware/include/wifi_config.example.h to firmware/include/wifi_config.h and fill it in"
#endif

#ifndef ROS_DOMAIN_ID
#define ROS_DOMAIN_ID 0
#endif

namespace uros {

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

namespace {

// ---- UDP transport --------------------------------------------------------
// Arduino's WiFiUDP splits datagrams at 1460 bytes. A raw lwIP socket sends each
// micro-ROS packet (up to the 2048 byte MTU in microros.meta) as one datagram, so a
// full LaserScan fits into a single best-effort message.

int sock = -1;
sockaddr_in agent_addr = {};

bool transport_open(uxrCustomTransport*) {
    sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return sock >= 0;
}

bool transport_close(uxrCustomTransport*) {
    if (sock >= 0) {
        lwip_close(sock);
    }
    sock = -1;
    return true;
}

size_t transport_write(uxrCustomTransport*, const uint8_t* buf, size_t len, uint8_t* err) {
    const int sent = lwip_sendto(sock, buf, len, 0, reinterpret_cast<sockaddr*>(&agent_addr), sizeof(agent_addr));
    if (sent < 0) {
        *err = 1;
        return 0;
    }
    return sent;
}

size_t transport_read(uxrCustomTransport*, uint8_t* buf, size_t len, int timeout_ms, uint8_t* err) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    timeval timeout = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (lwip_select(sock + 1, &fds, nullptr, nullptr, &timeout) <= 0) {
        return 0;
    }
    const int received = lwip_recv(sock, buf, len, 0);
    if (received < 0) {
        *err = 1;
        return 0;
    }
    return received;
}

// ---- Agent connection -----------------------------------------------------

const char* node_name = nullptr;
int led = -1;
bool connected = false;
uint32_t last_check_ms = 0;
uint32_t last_sync_ms = 0;

void teardown(void (*destroy)()) {
    // The agent is usually gone at this point: don't wait for it to confirm deletions.
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context != nullptr) {
        ignore(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));
    }
    destroy();
    ignore(rcl_node_fini(&node));
    ignore(rclc_support_fini(&support));
}

bool connect(bool (*create)(), void (*destroy)()) {
    allocator = rcl_get_default_allocator();
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) {
        return false;
    }
    const bool session_ok = rcl_init_options_set_domain_id(&options, ROS_DOMAIN_ID) == RCL_RET_OK &&
                            rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator) == RCL_RET_OK;
    ignore(rcl_init_options_fini(&options));
    if (!session_ok) {
        return false;
    }

    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, node_name, "", &support) == RCL_RET_OK && create()) {
        // Timestamps must be in ROS time, so a session only counts once the clock is synced.
        for (int i = 0; i < 3 && !rmw_uros_epoch_synchronized(); i++) {
            ignore(rmw_uros_sync_session(500));
        }
        if (rmw_uros_epoch_synchronized()) {
            return true;
        }
    }
    teardown(destroy);
    return false;
}

}  // namespace

void begin(const char* name, int status_led) {
    node_name = name;
    led = status_led;
    pinMode(led, OUTPUT);

    String hostname = String("roboost-") + name;
    hostname.replace('_', '-');
    WiFi.setHostname(hostname.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // power save adds 100+ ms latency to incoming packets
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[%s] connecting to WiFi '%s'", name, WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(led, !digitalRead(led));
        Serial.print('.');
        delay(250);
    }
    digitalWrite(led, LOW);
    Serial.printf("\n[%s] WiFi up, ip %s, agent %s:%d\n", name, WiFi.localIP().toString().c_str(), AGENT_IP, AGENT_PORT);

    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    agent_addr.sin_family = AF_INET;
    agent_addr.sin_port = htons(AGENT_PORT);
    agent_addr.sin_addr.s_addr = static_cast<uint32_t>(agent_ip);

    ignore(rmw_uros_set_custom_transport(false, nullptr, transport_open, transport_close, transport_write, transport_read));
}

bool spin(bool (*create)(), void (*destroy)()) {
    const uint32_t now = millis();
    if (!connected) {
        if (now - last_check_ms < 1000) {
            return false;
        }
        last_check_ms = now;
        digitalWrite(led, !digitalRead(led));  // slow blink: waiting for agent
        if (WiFi.status() != WL_CONNECTED || rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            return false;
        }
        if (!connect(create, destroy)) {
            Serial.printf("[%s] agent found, but session setup failed\n", node_name);
            return false;
        }
        connected = true;
        last_sync_ms = now;
        digitalWrite(led, HIGH);
        Serial.printf("[%s] connected to agent\n", node_name);
        return true;
    }

    if (now - last_check_ms >= 1000) {
        last_check_ms = now;
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            teardown(destroy);
            connected = false;
            digitalWrite(led, LOW);
            Serial.printf("[%s] agent lost\n", node_name);
            return false;
        }
        if (now - last_sync_ms >= 10000) {
            ignore(rmw_uros_sync_session(100));  // on failure the previous offset stays valid
            last_sync_ms = now;
        }
    }
    return true;
}

int64_t now_ns() { return rmw_uros_epoch_nanos(); }

void stamp(builtin_interfaces__msg__Time& t, int64_t ns) {
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

}  // namespace uros
