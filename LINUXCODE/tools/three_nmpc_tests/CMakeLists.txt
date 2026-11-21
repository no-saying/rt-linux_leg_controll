cmake_minimum_required(VERSION 3.16)
project(three_nmpc_model_checks LANGUAGES CXX)

enable_testing()

add_executable(three_nmpc_control_test test_decoupled_control.cpp)
target_link_libraries(three_nmpc_control_test PRIVATE eso_three_nmpc)
target_compile_options(three_nmpc_control_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
add_test(NAME three_nmpc_control_test COMMAND three_nmpc_control_test)

add_executable(three_nmpc_model_test test_decoupled_nmpc_model.cpp)
target_compile_features(three_nmpc_model_test PRIVATE cxx_std_17)
target_include_directories(three_nmpc_model_test PRIVATE
  ../../source/algorithms/eso_three_nmpc)
target_link_libraries(three_nmpc_model_test PRIVATE eso_three_nmpc)
target_compile_options(three_nmpc_model_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
add_test(NAME three_nmpc_model_test COMMAND three_nmpc_model_test)

add_executable(three_nmpc_application_binding_test test_application_binding.cpp)
target_link_libraries(three_nmpc_application_binding_test PRIVATE
  control_stack_application control_stack_algorithms)
target_compile_options(three_nmpc_application_binding_test PRIVATE
  -Wall -Wextra -Wpedantic -UNDEBUG)
add_test(NAME three_nmpc_application_binding_test
  COMMAND three_nmpc_application_binding_test)
