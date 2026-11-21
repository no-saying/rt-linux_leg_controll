cmake_minimum_required(VERSION 3.16)
project(eso_three_nmpc LANGUAGES CXX)

find_package(Threads REQUIRED)
add_library(eso_three_nmpc STATIC
  decoupled_control.cpp decoupled_nmpc.cpp serial_leg_dynamics.cpp
  decoupled_nmpc_qp.cpp acados_adapter.cpp)
target_compile_features(eso_three_nmpc PUBLIC cxx_std_17)
target_include_directories(eso_three_nmpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/control_stack/algorithms/eso_three_nmpc>)
target_link_libraries(eso_three_nmpc PUBLIC Threads::Threads)
if(CONTROL_STACK_VENDOR_OPTIMIZER)
  target_link_libraries(eso_three_nmpc PUBLIC acados_dense_qp_hpipm)
  target_include_directories(eso_three_nmpc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/optimizer/acados)
  target_compile_definitions(eso_three_nmpc PRIVATE NMPC_USE_HPIPM=1)
else()
  message(FATAL_ERROR "eso_three_nmpc requires the bundled acados dense-QP backend")
endif()
target_compile_options(eso_three_nmpc PRIVATE -Wall -Wextra -Wpedantic)
