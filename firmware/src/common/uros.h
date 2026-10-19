// micro-ROS over WiFi (plain UDP to the agent) and agent (re)connection handling,
// shared by both boards.
#pragma once

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <builtin_interfaces/msg/time.h>

namespace uros {

// Valid while connected.
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;

// Joins the WiFi from wifi_config.h (blocking) and points micro-ROS at the agent.
void begin(const char* node_name, int status_led);

// Call every loop(). Runs `create` once the agent is reachable (node already exists),
// `destroy` when it is lost. Returns true while connected and time-synced.
bool spin(bool (*create)(), void (*destroy)());

// rcl results we deliberately don't act on (best-effort publish, teardown).
// GCC ignores (void) casts for warn_unused_result, so pass them here instead.
inline void ignore(rcl_ret_t) {}

// Agent-synchronised wall clock.
int64_t now_ns();
void stamp(builtin_interfaces__msg__Time& t, int64_t ns = now_ns());

}  // namespace uros
