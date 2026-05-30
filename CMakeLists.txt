cmake_minimum_required(VERSION 3.16)
project(WeekendPilot CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 控制核：纯 C++，严格编译
file(GLOB_RECURSE CORE_SRC core/*.cpp)
add_library(wp_core STATIC ${CORE_SRC})
target_include_directories(wp_core PUBLIC core)
target_compile_options(wp_core PRIVATE -Wall -Wextra -Werror)

enable_testing()
add_subdirectory(test)
