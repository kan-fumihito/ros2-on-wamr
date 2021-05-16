
#include <wasm_c_api.h>
#include <wasm_export.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

int foo(wasm_exec_env_t exec_env, int a, int b)
{
    printf("Hello, Foo\n");
    return a + b;
}

void bar(wasm_exec_env_t exec_env, int a, int b, int c, int d)
{
    printf("Hello, Bar:%d\n", a + b + c + d);
}