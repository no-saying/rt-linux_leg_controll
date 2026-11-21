cmake_minimum_required(VERSION 3.16)
project(control_stack_board_architecture_test LANGUAGES CXX)
add_subdirectory(../source control_stack)
add_executable(control_stack_board_test src/main.cpp)
target_link_libraries(control_stack_board_test PRIVATE control_stack_application)
enable_testing()
add_test(NAME board_architecture_path COMMAND control_stack_board_test)
add_subdirectory(three_nmpc_tests)

