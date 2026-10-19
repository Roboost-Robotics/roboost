// Copy to wifi_config.h (git-ignored) and fill in. Used by both boards.
#pragma once

#define WIFI_SSID "my-network"
#define WIFI_PASSWORD "my-password"

// Machine running the micro-ROS agent (the dev container host).
#define AGENT_IP "192.168.1.100"
#define AGENT_PORT 8888

// Must match ROS_DOMAIN_ID on the host (default 0).
#define ROS_DOMAIN_ID 0
