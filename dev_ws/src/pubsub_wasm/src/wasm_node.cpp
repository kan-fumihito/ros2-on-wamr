// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <stdio.h>
#include <wasm_c_api.h>
#include <wasm_export.h>
#include "function.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

void init_wamr(void)
{
  /* initialize the wasm runtime by default configurations */
  wasm_runtime_init();

  // Define an array of NativeSymbol for the APIs to be exported.
  // Note: the array must be static defined since runtime
  //       will keep it after registration
  static NativeSymbol native_symbols[] =
      {
          {
              "bar",  // the name of WASM function name
              bar,    // the native function pointer
              "(iiii)" // the function prototype signature
          },
          {
              "foo",      // the name of WASM function name
              foo, // the native function pointer
              "(ii)i"     // the function prototype signature
          }};

  int n_native_symbols = sizeof(native_symbols) / sizeof(NativeSymbol);
  if (!wasm_runtime_register_natives("env",
                                     native_symbols,
                                     n_native_symbols))
  {
    printf("Error register natives\n");
  }
}

int main(int argc, char *argv[])
{
  FILE *fp;
  char *buffer, error_buf[128];
  wasm_module_t module;
  wasm_module_inst_t module_inst;
  wasm_function_inst_t func;
  wasm_exec_env_t exec_env;
  uint32_t size, stack_size = 8092, heap_size = 8092;

  fp = fopen(argv[1], "rb");
  fseek(fp, 0, SEEK_END);
  size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  buffer = malloc(sizeof(char) * size);
  fread(buffer, sizeof(char), size, fp);

  init_wamr();

  // natives registeration must be done before loading WASM modules
  module = wasm_runtime_load(buffer, size, error_buf, sizeof(error_buf));

  /* create an instance of the WASM module (WASM linear memory is ready) */
  module_inst = wasm_runtime_instantiate(module, stack_size, heap_size, error_buf, sizeof(error_buf));

  /* lookup a WASM function by its name
     The function signature can NULL here */
  func = wasm_runtime_lookup_function(module_inst, "ros_wasm", NULL);

  /* creat an execution environment to execute the WASM functions */
  exec_env = wasm_runtime_create_exec_env(module_inst, stack_size);

  uint32_t args[2] = {};

  /* arguments are always transferred in 32-bit element */

  /* call the WASM function */
  if (wasm_runtime_call_wasm(exec_env, func, 0, args))
  {
    /* the return value is stored in argv[0] */
    printf("wasm_main function return: %d\n", args[0]);
  }
  else
  {
    /* exception is thrown if call fails */
    printf("%s\n", wasm_runtime_get_exception(module_inst));
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
