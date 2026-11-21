cmake_minimum_required(VERSION 3.16)
project(wheel_control_stack LANGUAGES C CXX)

include(CTest)

option(CONTROL_STACK_VENDOR_OPTIMIZER "Build bundled acados/HPIPM/BLASFEO" ON)
set(CONTROL_STACK_BLASFEO_TARGET "GENERIC" CACHE STRING
  "BLASFEO target (GENERIC or an architecture supported by BLASFEO)")
if(CONTROL_STACK_VENDOR_OPTIMIZER)
  set(CMAKE_SKIP_INSTALL_RULES ON)
  set(TARGET "${CONTROL_STACK_BLASFEO_TARGET}" CACHE STRING "BLASFEO target" FORCE)
  set(LA "HIGH_PERFORMANCE" CACHE STRING "BLASFEO linear algebra backend" FORCE)
  set(MF "PANELMAJ" CACHE STRING "BLASFEO matrix format" FORCE)
  set(BLASFEO_HEADERS_INSTALLATION_DIRECTORY "blasfeo/include" CACHE STRING
    "BLASFEO headers in the control_stack package" FORCE)
  set(HPIPM_HEADERS_INSTALLATION_DIRECTORY "hpipm/include" CACHE STRING
    "HPIPM headers in the control_stack package" FORCE)
  set(BLASFEO_TESTING OFF CACHE BOOL "Disable bundled BLASFEO tests" FORCE)
  set(HPIPM_TESTING OFF CACHE BOOL "Disable bundled HPIPM tests" FORCE)
  set(BUILD_SHARED_LIBS OFF)
  add_subdirectory(third_party/optimizer/blasfeo EXCLUDE_FROM_ALL)
  set(BLASFEO_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/optimizer/blasfeo/include")
  add_subdirectory(third_party/optimizer/hpipm EXCLUDE_FROM_ALL)
  add_library(acados_dense_qp_hpipm STATIC
    third_party/optimizer/acados/acados/dense_qp/dense_qp_common.c
    third_party/optimizer/acados/acados/dense_qp/dense_qp_hpipm.c
    third_party/optimizer/acados/acados/utils/mem.c
    third_party/optimizer/acados/acados/utils/timing.c)
  target_include_directories(acados_dense_qp_hpipm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/optimizer/acados
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/optimizer/blasfeo/include
    ${CMAKE_CURRENT_BINARY_DIR}/third_party/optimizer/blasfeo/include)
  target_link_libraries(acados_dense_qp_hpipm PUBLIC hpipm blasfeo)
  set(CMAKE_SKIP_INSTALL_RULES OFF)
endif()

add_subdirectory(algorithms/eso_three_nmpc)

add_library(control_stack_common INTERFACE)
target_include_directories(control_stack_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_stack_common INTERFACE cxx_std_17)

add_library(control_stack_platform STATIC src/platform/amp/amp_frame.cpp)
target_link_libraries(control_stack_platform PUBLIC control_stack_common)
add_library(control_stack_algorithms STATIC src/algorithms/three_nmpc_adapter.cpp)
target_link_libraries(control_stack_algorithms PUBLIC control_stack_common eso_three_nmpc)
add_library(control_stack_application STATIC src/application/control_application.cpp src/application/control_runtime.cpp)
target_link_libraries(control_stack_application PUBLIC control_stack_algorithms control_stack_platform)
if(BUILD_TESTING)
  add_executable(acados_qp_solver_test
    tests/test_acados_qp_solver.cpp)
  target_include_directories(acados_qp_solver_test PRIVATE
    algorithms/eso_three_nmpc)
  target_link_libraries(acados_qp_solver_test PRIVATE eso_three_nmpc)
  target_compile_options(acados_qp_solver_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME acados_qp_solver_test COMMAND acados_qp_solver_test)

  add_executable(three_nmpc_adapter_test tests/test_three_nmpc_adapter.cpp)
  target_link_libraries(three_nmpc_adapter_test PRIVATE control_stack_algorithms)
  target_compile_options(three_nmpc_adapter_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME three_nmpc_adapter_test COMMAND three_nmpc_adapter_test)
endif()
if(NOT MSVC)
  target_link_libraries(control_stack_platform PUBLIC m)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS control_stack_common control_stack_algorithms control_stack_platform control_stack_application eso_three_nmpc acados_dense_qp_hpipm hpipm blasfeo
  EXPORT ControlStackTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY algorithms/eso_three_nmpc/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/control_stack/algorithms/eso_three_nmpc
  FILES_MATCHING PATTERN "*.h")
install(DIRECTORY third_party/optimizer/blasfeo/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/blasfeo/include)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/third_party/optimizer/blasfeo/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/blasfeo/include)
install(DIRECTORY third_party/optimizer/hpipm/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpipm/include)
install(DIRECTORY third_party/optimizer/acados/acados/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/acados
  FILES_MATCHING PATTERN "*.h")
install(FILES third_party/optimizer/acados/LICENSE.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/ControlStack
  RENAME acados-LICENSE.txt)
install(FILES third_party/optimizer/hpipm/LICENSE.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/ControlStack
  RENAME hpipm-LICENSE.txt)
install(FILES third_party/optimizer/blasfeo/LICENSE.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/ControlStack
  RENAME blasfeo-LICENSE.txt)
install(EXPORT ControlStackTargets
  FILE ControlStackTargets.cmake
  NAMESPACE control_stack::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ControlStack)
configure_package_config_file(cmake/ControlStackConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ControlStackConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ControlStack)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ControlStackConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ControlStack)


