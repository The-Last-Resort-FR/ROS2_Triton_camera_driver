cmake_minimum_required(VERSION 3.5)
project(lucid_cam_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_CXX_FLAGS "-g3 -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindARENA.cmake")


link_directories(${arena_sdk_LIBRARIES})

ament_auto_find_build_dependencies()



ament_auto_add_library(lucid_cam_driver SHARED
        "include/lucid_cam_driver/benchmark.hpp"
        "include/lucid_cam_driver/camera.hpp"
        "include/lucid_cam_driver/cameraMaster.hpp"
        "include/lucid_cam_driver/logger.hpp"
        "include/lucid_cam_driver/serial.hpp"
        "src/lucid_cam.cpp"
        "src/benchmark.cpp"
        "src/camera.cpp"
        "src/cameraMaster.cpp"
        "src/logger.cpp"
        "src/serial.cpp")

ament_target_dependencies(lucid_cam_driver
        "rclcpp"
        "std_msgs"
        "sensor_msgs"
        "cv_bridge"
        "image_transport"
        "rclcpp_components")

target_link_libraries(lucid_cam_driver
    serial
)

## ArenaSDK is not handled by ament, so we need the manual steps below.
target_include_directories(lucid_cam_driver
        SYSTEM
        PUBLIC
        ${arena_sdk_INCLUDES}
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(lucid_cam_driver
        ${OpenCV_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        )

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.
ament_export_include_directories(${arena_sdk_INCLUDES})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()


ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        param
        )
find_package(PkgConfig REQUIRED)
        pkg_check_modules(FFMPEG REQUIRED 
            libavcodec 
            libavformat 
            libavutil 
            libswscale
        )
        
target_include_directories(lucid_cam_driver
        SYSTEM PUBLIC
        ${FFMPEG_INCLUDE_DIRS}
)

target_link_libraries(lucid_cam_driver
        ${FFMPEG_LIBRARIES}
)

ament_auto_add_executable(lucid_cam_node
        src/lucid_cam.cpp
    )
    
target_link_libraries(lucid_cam_node
        lucid_cam_driver
    )
    
install(TARGETS lucid_cam_node
        DESTINATION lib/${PROJECT_NAME}
    )