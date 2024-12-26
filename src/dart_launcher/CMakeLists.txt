

cmake_minimum_required(VERSION 3.18)
project(dart_launcher)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dart_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 查找 GSL 库
find_package(GSL REQUIRED)

find_package(yaml-cpp REQUIRED)


include_directories(include)
# 看门狗
add_executable(node_dart_logger_dog src/node_dart_logger_dog.cpp src/dart_config.cpp)
ament_target_dependencies(node_dart_logger_dog rclcpp std_msgs dart_msgs std_srvs)
# Add node_lvgl_ui target
add_executable(node_lvgl_ui src/node_lvgl_ui.cpp src/get_ip.cpp src/dart_config.cpp src/dart_algorithm.cpp)
ament_target_dependencies(node_lvgl_ui rclcpp std_msgs dart_msgs std_srvs sensor_msgs cv_bridge)

# Add Lvgl support
target_include_directories(node_lvgl_ui PRIVATE
    ${PROJECT_SOURCE_DIR}
    include
    src/generated src/custom src/generated/guider_customer_fonts src/generated/guider_fonts src/generated/images
    ${OpenCV_INCLUDE_DIRS}
    lvgl/include/lvgl
)

# 递归添加子目录${PROJECT_SOURCE_DIR}/generated/*
FILE(GLOB_RECURSE SOURCES src/custom/*.c src/custom/*.cpp /src/dart_config.cpp)
target_sources(node_lvgl_ui PRIVATE ${SOURCES})
target_link_directories(node_lvgl_ui PRIVATE lvgl/lib src/generated/lib)

if(BUILD_SHARED_LIBS)
    target_link_libraries(node_lvgl_ui lvgl lv_drivers guider_generated ${OpenCV_LIBS} yaml-cpp ${GSL_LIBRARIES})
else()
    target_link_libraries(node_lvgl_ui liblvgl.a liblv_drivers.a libguider_generated.a ${OpenCV_LIBS} yaml-cpp ${GSL_LIBRARIES})
endif()

target_link_libraries(node_lvgl_ui)

# Add node_can_agent target
add_executable(node_can_agent src/node_can_agent.cpp src/dart_config.cpp)
ament_target_dependencies(node_can_agent rclcpp std_msgs dart_msgs std_srvs)

# Add libsockcanpp support
target_include_directories(node_can_agent PRIVATE libsockcanpp/include libsockcanpp/include/exceptions include)

# libsocketcancpp/lib/libsocketcancpp.so
target_link_directories(node_can_agent PRIVATE libsockcanpp/lib)

if(BUILD_SHARED_LIBS)
    target_link_libraries(node_can_agent socketcancpp)
else()
    target_link_libraries(node_can_agent libsocketcancpp.a)
endif()

target_link_libraries(node_can_agent)

# Add node_dart_config target
add_executable(node_dart_config src/node_dart_config.cpp src/dart_config.cpp)
target_include_directories(node_dart_config PRIVATE include)

# yaml lib
target_link_libraries(node_dart_config yaml-cpp ${GSL_LIBRARIES})

# target_link_libraries(node_can_agent yaml-cpp)
ament_target_dependencies(node_dart_config rclcpp std_msgs dart_msgs std_srvs)

# Set the path to the YAML file relative to the ROS 2 install directory
# set(YAML_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/")
set(YAML_PATH "${CMAKE_CURRENT_LIST_DIR}/config/")

# Pass the YAML path as a definition with double quotes
add_definitions(-DYAML_PATH=\"${YAML_PATH}\")

# Dart Calibration Config
add_executable(dart_calibration src/dart_calibration.cpp src/dart_algorithm.cpp)
target_link_libraries(dart_calibration yaml-cpp ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES})

install(TARGETS
    node_lvgl_ui
    node_can_agent
    node_dart_config
    node_dart_logger_dog
    dart_calibration
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_SHARED_LIBS)
    # 提取所有的.so，不要保留/lib路径
    install(DIRECTORY
        lvgl/lib/
        DESTINATION lib
        FILES_MATCHING PATTERN "*.so"
    )

    install(DIRECTORY
        libsockcanpp/lib/
        DESTINATION lib
        FILES_MATCHING PATTERN "*.so"
    )

    install(DIRECTORY
        src/generated/lib/
        DESTINATION lib
        FILES_MATCHING PATTERN "*.so"
    )
endif()

ament_package()

# Ensure compile_commands.json is generated
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)