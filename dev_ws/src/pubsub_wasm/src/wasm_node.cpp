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
          "create_node_wasm",
          create_node_wasm,
          "(*~)i"
        },
        {
          "create_publisher_wasm",
          create_publisher_wasm,
          "(i*~)i"
        },
        {
          "publish_wasm",
          publish_wasm,
          "(i*~)"
        },
        {
          "wait_1000ms",
          wait_1000ms,
          "()"
        },
        {
          "spin_wasm",
          spin_wasm,
          "(i)"
        }
      };

  int n_native_symbols = sizeof(native_symbols) / sizeof(NativeSymbol);
  if (!wasm_runtime_register_natives("env",
                                     native_symbols,
                                     n_native_symbols))
  {
    printf("Error register natives\n");
  }
}

#include <wasm.h>
#include <ros.hpp>

int main(argc, *argv[]){
  char *wasm_bin = read_file(argv[1]);

  rclcpp::init();

  auto env = init_wasm();

  wasm_runtime(wasm_bin, env);

  rclcpp::shutdown();

  return 0;
}

int main(int argc, char *argv[])
{
  FILE *fp;
  char error_buf[128];
  wasm_module_t module;
  wasm_module_inst_t module_inst;
  wasm_function_inst_t func;
  wasm_exec_env_t exec_env;
  uint32_t size, stack_size = 8092, heap_size = 8092;

  fp = fopen(argv[1], "rb");
  fseek(fp, 0, SEEK_END);
  size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  uint8_t *buffer = (uint8_t*)malloc(sizeof(char) * size);
  fread(buffer, sizeof(char), size, fp);

  rclcpp::init(argc, argv);
  init_wamr();

  module = wasm_runtime_load(buffer, size, error_buf, sizeof(error_buf));
  module_inst = wasm_runtime_instantiate(module, stack_size, heap_size, error_buf, sizeof(error_buf));
  func = wasm_runtime_lookup_function(module_inst, "ros_main", NULL);
  exec_env = wasm_runtime_create_exec_env(module_inst, stack_size);
  a = wasm_runtime_call(instance, "ros_main");
  uint32_t args[2] = {};

  if(func==NULL){
	  printf("Func is NULL\n");
	  return 1;
  }
  if (wasm_runtime_call_wasm(exec_env, func, 0, args))
  {
    printf("wasm_main function return: %d\n", args[0]);
  }
  else
  {
    printf("%s\n", wasm_runtime_get_exception(module_inst));
  }

  return 0;
}
