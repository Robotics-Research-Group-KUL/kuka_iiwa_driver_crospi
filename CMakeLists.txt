cmake_minimum_required(VERSION 3.8)
project(kuka_iiwa_driver)

message("--------------helloooo----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(etasl_ros2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

## KUKA SDK specific
# TODO: Modify proper directories, since this CMakeList is not anymore in src folder
set(KUKA_FRI_SDK ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/)
set(FRISDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/include)
set(FRISDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/lib)
add_custom_target(kuka_fri_sdk COMMAND make clean && make
                   WORKING_DIRECTORY ${KUKA_FRI_SDK}/build/GNUMake
)
include_directories(${FRISDK_INCLUDE})

set(src_kuka_iiwa_driver
    src/kukaiiwa_robotdriver.cpp
    src/kukaiiwa_robotdriverfactory.cpp
    src/kukaiiwa_client.cpp
)

add_library(kuka_iiwa_driver ${src_kuka_iiwa_driver})

add_dependencies(kuka_iiwa_driver kuka_fri_sdk) #Makes sure to compile first kuka_fri_sdk before the kuka_iiwa_driver

find_library(FRI_CLIENT_LIB
             NAMES libFRIClient.so
             PATHS ${FRISDK_LIB}
)

set(rosdependencies)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
    etasl_misc_lib 
    etasl_task_utils
    etasl_node_utils
    etasl_interfacing_utils
    # ${FRISDK_LIB}/libFRIClient.so
    ${FRI_CLIENT_LIB}
    )



message("--------------FRI_CLIENT_LIB: ${FRI_CLIENT_LIB}----------------")


ament_target_dependencies(kuka_iiwa_driver
  "etasl_ros2"
  "pluginlib"
)

pluginlib_export_plugin_description_file(etasl_ros2 plugins.xml)


target_include_directories(kuka_iiwa_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(kuka_iiwa_driver ${rosdependencies})  
target_link_libraries(kuka_iiwa_driver ${dependencies})

install(TARGETS kuka_iiwa_driver
  DESTINATION lib/${PROJECT_NAME})


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

ament_package()
