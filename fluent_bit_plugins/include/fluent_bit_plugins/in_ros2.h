/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

/*  Fluent Bit
 *  ==========
 *  Copyright (C) 2015-2022 The Fluent Bit Authors
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef FLB_IN_ROS2_H
#define FLB_IN_ROS2_H

#include <fluent-bit/flb_config.h>
#include <fluent-bit/flb_config_map.h>
#include <fluent-bit/flb_error.h>
#include <fluent-bit/flb_info.h>
#include <fluent-bit/flb_input.h>
#include <fluent-bit/flb_input_plugin.h>
#include <fluent-bit/flb_pack.h>
#include <fluent-bit/flb_time.h>
#include <msgpack.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "dc_interfaces/msg/string_stamped.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"

struct rclc_subscriber
{
  rcl_subscription_t data_subscription;
  const rosidl_message_type_support_t* data_type_support;
  rcl_subscription_options_t subscription_options;
  char* topic_name;
  dc_interfaces__msg__StringStamped data_msg;
  struct mk_list _head;
};

/* ROS2 Input configuration & context */
struct flb_ros2
{
  struct mk_list* topics_list; /* Topic names as a list */
  struct mk_list topic_subs;   /* topics subscribers */
  flb_sds_t node_name;         /* Name of the node      */
  char* topics;                /* Topic names as a string */
  int spin_time;               /* Time to wait, in ms   */
  int buf_len;                 /* read buffer length    */
  char* buf;                   /* read buffer           */
  struct flb_pack_state pack_state;
  struct flb_input_instance* ins;
};

extern struct flb_input_plugin in_ros2_plugin;

#endif
