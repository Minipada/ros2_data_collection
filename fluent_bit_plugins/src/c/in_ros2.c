#include "fluent_bit_plugins/in_ros2.h"

// Ideally we pass pointers and not use global variables
rclc_executor_t EXECUTOR;
rcl_node_t NODE;
rcl_init_options_t INIT_OPTIONS;
rcl_context_t CONTEXT;
rcl_allocator_t ALLOCATOR;
rcl_node_options_t NODE_OPS;
void* GLOB_CTX;
static volatile bool NEW_DATA_FLAG = false;

static int config_destroy(struct flb_ros2* ctx)
{
  // Destroy topic_subs
  struct mk_list* tmp;
  struct mk_list* head;
  struct rclc_subscriber* subscriber;
  mk_list_foreach_safe(head, tmp, &ctx->topic_subs)
  {
    subscriber = mk_list_entry(head, struct rclc_subscriber, _head);
    mk_list_del(&subscriber->_head);
    flb_free(subscriber);
  }

  // Destroy topics_list
  struct mk_list* topics_list = (struct mk_list*)ctx->topics_list;
  struct flb_slist_entry* entry;

  if (topics_list != NULL)
  {
    mk_list_foreach_safe(head, tmp, topics_list)
    {
      entry = mk_list_entry(head, struct flb_slist_entry, _head);
      if (entry != NULL)
      {
        mk_list_del(&entry->_head);
        flb_free(entry);
      }
    }
  }

  flb_free(topics_list);

  // Free ctx
  flb_free(ctx);
  return 0;
}

static int set_timestamp(msgpack_packer* mp_pck, const dc_interfaces__msg__StringStamped* msg)
{
  struct flb_time msg_time = { .tm.tv_sec = msg->header.stamp.sec, .tm.tv_nsec = msg->header.stamp.nanosec };
  int ret = flb_time_append_to_msgpack(&msg_time, mp_pck, 0);

  return ret;
}

static inline int process_pack(msgpack_packer* mp_pck, struct flb_ros2* ctx, char* data, size_t data_size,
                               dc_interfaces__msg__StringStamped* msg)
{
  size_t off = 0;
  msgpack_unpacked result;
  msgpack_object entry;

  /* Queue the data with time field */
  msgpack_unpacked_init(&result);

  while (msgpack_unpack_next(&result, data, data_size, &off) == MSGPACK_UNPACK_SUCCESS)
  {
    entry = result.data;

    if (entry.type == MSGPACK_OBJECT_MAP)
    {
      flb_debug("MSGPACK_OBJECT_MAP");
      msgpack_pack_array(mp_pck, 2);
      set_timestamp(mp_pck, msg);
      msgpack_pack_object(mp_pck, entry);
    }
    else if (entry.type == MSGPACK_OBJECT_ARRAY)
    {
      flb_debug("MSGPACK_OBJECT_ARRAY");
      msgpack_pack_object(mp_pck, entry);
    }
    else
    {
      /*
       * Upon exception, acknowledge the user about the problem but continue
       * working, do not discard valid JSON entries.
       */
      flb_error("invalid record found, it's not a JSON map or array");
      msgpack_unpacked_destroy(&result);
      return -1;
    }
  }

  msgpack_unpacked_destroy(&result);
  return 0;
}
void data_callback(const void* msgin)
{
  dc_interfaces__msg__StringStamped* msg = (dc_interfaces__msg__StringStamped*)msgin;
  if (msg == NULL)
  {
    flb_warn("Callback: msg NULL\n");
  }
  else
  {
    flb_debug("Callback: I heard: ts=%d.%d data=%s \n", msg->header.stamp.sec, msg->header.stamp.nanosec,
              msg->data.data);
    struct flb_ros2* ctx = GLOB_CTX;

    msgpack_sbuffer mp_sbuf;
    msgpack_unpacked result;
    msgpack_packer mp_pck;
    int root_type;

    /* Queue the data with time field */
    msgpack_unpacked_init(&result);

    /* Initialize local msgpack buffer */
    msgpack_sbuffer_init(&mp_sbuf);
    msgpack_packer_init(&mp_pck, &mp_sbuf, msgpack_sbuffer_write);

    size_t pack_size;
    char* pack;
    ctx->buf = msg->data.data;
    ctx->buf_len = msg->data.capacity;
    flb_debug("buf_len: %ld, buf: '%s'", ctx->buf_len, ctx->buf);

    int ret = flb_pack_json(ctx->buf, ctx->buf_len, &pack, &pack_size, &root_type);
    if (ret == FLB_ERR_JSON_PART)
    {
      flb_warn("Data incomplete, waiting for more...");
      msgpack_sbuffer_destroy(&mp_sbuf);
    }
    else if (ret == FLB_ERR_JSON_INVAL)
    {
      flb_warn("Invalid JSON message, skipping");
      msgpack_sbuffer_destroy(&mp_sbuf);
    }
    else if (ret == 0)
    {
      flb_debug("Data complete");
      /* Process valid packaged records */
      process_pack(&mp_pck, ctx, pack, pack_size, msg);

      flb_pack_state_reset(&ctx->pack_state);
      flb_pack_state_init(&ctx->pack_state);
      flb_free(pack);

      flb_input_log_append(ctx->ins, NULL, 0, mp_sbuf.data, mp_sbuf.size);
      msgpack_sbuffer_destroy(&mp_sbuf);
    }
  }
}

static int ros2_rclc_init()
{
  /* Initialize rclc */
  ALLOCATOR = rcl_get_default_allocator();
  NODE_OPS = rcl_node_get_default_options();

  /* Define ROS context */
  CONTEXT = rcl_get_zero_initialized_context();

  /* Create init_options */
  rcl_ret_t rc = rcl_init_options_init(&INIT_OPTIONS, ALLOCATOR);
  if (rc != RCL_RET_OK)
  {
    flb_error("Error rcl_init_options_init: %d.\n", rc);
    return -1;
  }

  /* Create context */
  rc = rcl_init(0, NULL, &INIT_OPTIONS, &CONTEXT);
  if (rc != RCL_RET_OK)
  {
    flb_error("Error in rcl_init.\n");
    return -1;
  }
  return 0;
}

static int ros2_node_init(struct flb_ros2* ctx)
{
  /* Create node */
  NODE = rcl_get_zero_initialized_node();
  const char* node_name = ctx->node_name;
  rcl_ret_t rc = rcl_node_init(&NODE, node_name, "/", &CONTEXT, &NODE_OPS);
  if (rc != RCL_RET_OK)
  {
    flb_error("Error in rclc_node_init\n");
    return -1;
  }
  else
  {
    flb_info("Started node %s", node_name);
  }
  return 0;
}

static int ros2_subscribers_init(struct flb_ros2* ctx)
{
  struct mk_list* head;
  struct rclc_subscriber* subscriber;
  struct flb_slist_entry* topic = NULL;

  int len = mk_list_size(ctx->topics_list);
  if (!ctx->topics_list || len == 0)
  {
    flb_error("No 'topics' options has been specified.");
    return 0;
  }
  mk_list_init(&ctx->topic_subs);

  mk_list_foreach(head, ctx->topics_list)
  {
    topic = mk_list_entry(head, struct flb_slist_entry, _head);
    subscriber = flb_malloc(sizeof(struct rclc_subscriber));
    if (!subscriber)
    {
      flb_errno();
      return -1;
    }
    subscriber->data_subscription = rcl_get_zero_initialized_subscription();
    subscriber->data_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(dc_interfaces, msg, StringStamped);
    subscriber->subscription_options = rcl_subscription_get_default_options();

    rcl_ret_t rc = rcl_subscription_init(&(subscriber->data_subscription), &NODE, subscriber->data_type_support,
                                         topic->str, &(subscriber->subscription_options));
    if (rc != RCL_RET_OK)
    {
      flb_error("Failed to create subscriber %s.\n", topic->str);
      return -1;
    }
    else
    {
      flb_info("Created subscriber %s", topic->str);
    }
    if (false == dc_interfaces__msg__StringStamped__init(&(subscriber->data_msg)))
    {
      flb_error("Failed to init msg.\n");
      return -1;
    }

    mk_list_add(&subscriber->_head, &(ctx->topic_subs));
  }

  return 0;
}

static int ros2_executor_init(struct flb_ros2* ctx)
{
  /* Executor */
  int len = mk_list_size(ctx->topics_list);
  rclc_executor_init(&EXECUTOR, &CONTEXT, len, &ALLOCATOR);

  /* Add subscriptions to executor */
  struct mk_list* head;
  struct mk_list* tmp;
  struct rclc_subscriber* an_item;
  rcl_ret_t rc;

  mk_list_foreach_safe(head, tmp, &ctx->topic_subs)
  {
    an_item = mk_list_entry(head, struct rclc_subscriber, _head);
    rc = rclc_executor_add_subscription(&EXECUTOR, &an_item->data_subscription, &(an_item->data_msg), &data_callback,
                                        ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
      flb_error("Error in rclc_executor_add_subscription.\n");
    }
  }
  rc = rclc_executor_prepare(&EXECUTOR);
  if (rc != RCL_RET_OK)
  {
    flb_error("Error in rclc_executor_prepare.\n");
  }

  return 0;
}

/* cb_collect callback */
static int in_ros2_collect(struct flb_input_instance* ins, struct flb_config* config, void* in_context)
{
  struct flb_ros2* ctx = in_context;
  rclc_executor_spin_some(&EXECUTOR, RCL_MS_TO_NS(ctx->spin_time));
  return 0;
}

/* Initialize plugin */
static int in_ros2_init(struct flb_input_instance* in, struct flb_config* config, void* data)
{
  int ret = -1;
  struct flb_ros2* ctx;

  /* Allocate space for the configuration */
  ctx = flb_calloc(1, sizeof(struct flb_ros2));
  if (!ctx)
  {
    return -1;
  }
  ctx->ins = in;

  const char* node_name;

  node_name = (char*)flb_input_get_property("node_name", in);
  if (node_name == NULL)
  {
    ctx->node_name = "fluentbit_rclc";
  }
  else
  {
    ctx->node_name = (char*)node_name;
  }
  const char* spin_time;
  spin_time = flb_input_get_property("spin_time", in);
  if (spin_time == NULL)
  {
    ctx->spin_time = 100;
  }
  else
  {
    ctx->spin_time = atoi(spin_time);
  }

  ctx->topics_list = flb_malloc(sizeof(struct mk_list));
  if (!ctx->topics_list)
  {
    flb_errno();
    return -1;
  }
  mk_list_init(ctx->topics_list);

  const char* topics;
  topics = flb_input_get_property("topics", in);
  ctx->topics = (char*)topics;
  int max_split = 0;
  ret = flb_slist_split_tokens(ctx->topics_list, ctx->topics, max_split);

  if (ret == -1)
  {
    config_destroy(ctx);
    return -1;
  }

  ret = ros2_rclc_init();
  if (ret == -1)
  {
    config_destroy(ctx);
    return -1;
  }

  ret = ros2_node_init(ctx);
  if (ret == -1)
  {
    config_destroy(ctx);
    return -1;
  }

  ret = ros2_subscribers_init(ctx);
  if (ret == -1)
  {
    config_destroy(ctx);
    return -1;
  }

  ret = ros2_executor_init(ctx);
  if (ret == -1)
  {
    config_destroy(ctx);
    return -1;
  }

  GLOB_CTX = ctx;
  /* Always initialize built-in JSON pack state */
  flb_pack_state_init(&ctx->pack_state);
  /* Load fluentbit context config */
  flb_input_set_context(in, ctx);
  /* Fluentbit collect */
  ret = flb_input_set_collector_time(in, in_ros2_collect, 0, 1000000, config);
  if (ret < 0)
  {
    flb_error("Could not set collector for ros2 input plugin");
    flb_free(ctx->topics_list);
    return -1;
  }

  return 0;
}

static int in_ros2_exit(void* data, struct flb_config* config)
{
  /* Clean up */
  rcl_ret_t rc = RCL_RET_OK;
  struct rclc_subscriber* an_item;
  struct mk_list* head;
  struct mk_list* tmp;
  struct flb_ros2* ctx = data;

  mk_list_foreach_safe(head, tmp, &ctx->topic_subs)
  {
    an_item = mk_list_entry(head, struct rclc_subscriber, _head);
    rc = rcl_subscription_fini(&an_item->data_subscription, &NODE);
  }
  rc += rcl_node_fini(&NODE);
  rc += rcl_init_options_fini(&INIT_OPTIONS);
  mk_list_foreach_safe(head, tmp, &ctx->topic_subs)
  {
    an_item = mk_list_entry(head, struct rclc_subscriber, _head);
    dc_interfaces__msg__StringStamped__fini(&(an_item->data_msg));
  }
  rc += rclc_executor_fini(&EXECUTOR);

  if (rc != RCL_RET_OK)
  {
    flb_error("Error while cleaning up!\n");
    return -1;
  }

  config_destroy(ctx);
  return 0;
}

struct flb_input_plugin IN_ROS2_PLUGIN = { .name = "ros2",
                                           .description = "ROS2 Input",
                                           .cb_init = in_ros2_init,
                                           .cb_pre_run = NULL,
                                           .cb_collect = in_ros2_collect,
                                           .cb_flush_buf = NULL,
                                           .cb_exit = in_ros2_exit };
