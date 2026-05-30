cmake_minimum_required(VERSION 3.16)
project(WeekendPilot CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 控制核：纯 C++，严格编译
file(GLOB_RECURSE CORE_SRC core/*.cpp)
add_library(wp_core STATIC ${CORE_SRC})
target_include_directories(wp_core PUBLIC core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(wp_core PRIVATE -Wall -Wextra -Werror)
elseif(MSVC)
  target_compile_options(wp_core PRIVATE /W4 /WX)
endif()

enable_testing()
add_subdirectory(test)

# SITL 共享库：供 Python ctypes 加载
add_library(wp_core_capi SHARED hal_sitl/core_c_api.cpp)
target_link_libraries(wp_core_capi PRIVATE wp_core)
target_include_directories(wp_core_capi PRIVATE core hal_sitl)
