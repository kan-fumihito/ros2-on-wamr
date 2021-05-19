
#include <wasm_c_api.h>
#include <wasm_export.h>
#include <stdio.h>
#include <map>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::map;

typedef uint32_t u32;
struct RosData
{
  map<u32, std::shared_ptr<rclcpp::Node>> nodes;
  map<u32, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>> publishers;
};

struct RosData ros_data;

u32 create_node_wasm(wasm_exec_env_t exec_env, char *s, u32 len)
{
    char node_name[16]={};
    strncpy(node_name, s, len);
    printf("node name: `%s`\n", node_name);
    auto node = rclcpp::Node::make_shared(node_name);
  u32 id = 0;
  while(ros_data.nodes.count(id)>0){
    id++;
  }
  ros_data.nodes.insert(make_pair(id,node));
  return id;
}

u32 create_publisher_wasm(wasm_exec_env_t exec_env, u32 node_id, char *s, u32 len)
{
    char topic_name[16] = {};
    strncpy(topic_name, s, len);
  auto node = ros_data.nodes[node_id];
  auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name, 10);
  u32 id = 0;
  while(ros_data.publishers.count(id)>0){
    id++;
  }
  ros_data.publishers.insert(make_pair(id,publisher));
  return id;
}

void publish_wasm(wasm_exec_env_t exec_env, u32 publisher_id, char* s, u32 len)
{
    char *msg=(char*)malloc(len);
    strncpy(msg, s, len);
  std_msgs::msg::String message;
  auto publisher = ros_data.publishers[publisher_id];

  message.data = msg;
  publisher->publish(message);
  printf("Publish: %s\n", msg);
}

void spin_wasm(wasm_exec_env_t exec_env, u32 node_id){
    auto node = ros_data.nodes[node_id];
    rclcpp::spin_some(node);
}

void wait_1000ms(wasm_exec_env_t exec_env){
    sleep(1);
}