cmake_minimum_required(VERSION 3.10)
project(kuka_iiwa_driver_crospi)

message("--------------kuka_iiwa_driver_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# crospi_core exports all its dependencies (rclcpp, pluginlib, fmt, jsoncpp, ...), so they are found and linked
# through crospi_core::crospi_core. Only the KUKA FRI SDK is added here.
find_package(crospi_core REQUIRED)

# ----------------------------------------------------------------------------------------------
# KUKA FRI SDK (closed source, obtained from KUKA, see README.md). By default in external/kuka_fri_sdk;
# another location can be given with -DKUKA_FRI_SDK_DIR=<dir>. The SDK library (libFRIClient.so) is built
# with its own GNUMake build (only once, the result is kept in <sdk>/lib) and installed with this package.
# ----------------------------------------------------------------------------------------------
set(KUKA_FRI_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk" CACHE PATH "Directory of the KUKA FRI SDK")
if(NOT EXISTS "${KUKA_FRI_SDK_DIR}/include/friClientApplication.h" OR NOT EXISTS "${KUKA_FRI_SDK_DIR}/build/GNUMake/Makefile")
  message(FATAL_ERROR "KUKA FRI SDK not found in KUKA_FRI_SDK_DIR=${KUKA_FRI_SDK_DIR}. Place the SDK provided by KUKA "
                      "in external/kuka_fri_sdk (see README.md) or pass -DKUKA_FRI_SDK_DIR=<dir>.")
endif()
message(STATUS "KUKA FRI SDK: ${KUKA_FRI_SDK_DIR}")

set(FRI_CLIENT_LIBRARY "${KUKA_FRI_SDK_DIR}/lib/libFRIClient.so")
add_custom_command(
  OUTPUT ${FRI_CLIENT_LIBRARY}
  COMMAND make lib
  WORKING_DIRECTORY ${KUKA_FRI_SDK_DIR}/build/GNUMake
  COMMENT "Building the KUKA FRI SDK library (libFRIClient.so)"
)
add_custom_target(kuka_fri_sdk DEPENDS ${FRI_CLIENT_LIBRARY})

add_library(kuka_fri_client SHARED IMPORTED)
# The SDK Makefile builds libFRIClient.so without SONAME: IMPORTED_NO_SONAME links it as -lFRIClient, so the plugin
# looks it up by name (in the install lib directory) instead of by the absolute path of the SDK in the source tree.
set_target_properties(kuka_fri_client PROPERTIES
  IMPORTED_NO_SONAME TRUE
  IMPORTED_LOCATION ${FRI_CLIENT_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES ${KUKA_FRI_SDK_DIR}/include)
add_dependencies(kuka_fri_client kuka_fri_sdk)

# ----------------------------------------------------------------------------------------------
# Plugin library
# ----------------------------------------------------------------------------------------------
add_library(kuka_iiwa_driver_crospi SHARED
    src/kuka_iiwa_driver_crospi.cpp
    src/kukaiiwa_client.cpp
)

target_include_directories(kuka_iiwa_driver_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(kuka_iiwa_driver_crospi PUBLIC
  crospi_core::crospi_core
  kuka_fri_client)

pluginlib_export_plugin_description_file(crospi_core plugins.xml)

ament_export_dependencies(crospi_core)

install(FILES ${FRI_CLIENT_LIBRARY} DESTINATION lib)

install(
  TARGETS kuka_iiwa_driver_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(
#   include
# )

ament_package()
