cmake_minimum_required(VERSION 2.8.3)
project(nlink_parser)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_subdirectory(extern/serial)
include_directories(extern/serial/include)

# find_package(serial CONFIG REQUIRED)

# # Generate messages in the 'msg' folder
add_message_files(
    FILES
    LinktrackAnchorframe0.msg
    LinktrackNode0.msg
    LinktrackNode1.msg
    LinktrackNode2.msg
    LinktrackNodeframe0.msg
    LinktrackNodeframe1.msg
    LinktrackNodeframe2.msg
    LinktrackNodeframe3.msg
    LinktrackTag.msg
    LinktrackTagframe0.msg
    TofsenseCascade.msg
    TofsenseFrame0.msg
    LinktrackAoaNode0.msg
    LinktrackAoaNodeframe0.msg
    LinktrackNode4Anchor.msg
    LinktrackNode4Tag.msg
    LinktrackNodeframe4.msg
    LinktrackNode5.msg
    LinktrackNodeframe5.msg
    LinktrackNode6.msg
    LinktrackNodeframe6.msg
    TofsenseMFrame0.msg
    TofsenseMFrame0Pixel.msg
    IotFrame0.msg
    IotFrame0Node.msg
)

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(

    # INCLUDE_DIRS include
    # LIBRARIES nlink
    CATKIN_DEPENDS message_runtime

    # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    src/utils
)

set(nutils "nutils")
add_library(${nutils}
    src/utils/protocol_extracter/nprotocol_base.cpp
    src/utils/protocol_extracter/nprotocol_extracter.cpp
    src/utils/nlink_unpack/nlink_utils.c
    src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
    src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
    src/utils/nlink_unpack/nlink_tofsense_frame0.c
    src/utils/nlink_unpack/nlink_tofsensem_frame0.c
    src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
    src/utils/nlink_unpack/nlink_iot_frame0.c
    src/utils/init_serial.cpp
    src/utils/nlink_protocol.cpp
    src/utils/nutils.cpp
)
target_link_libraries(${nutils}
    ${catkin_LIBRARIES}
    serial
)

set(LINKTRACK_NAME "linktrack")
add_executable(${LINKTRACK_NAME}
    src/linktrack/init.cpp
    src/linktrack/main.cpp
    src/linktrack/protocols.cpp
)
target_link_libraries(${LINKTRACK_NAME}

    # ${catkin_LIBRARIES}
    ${nutils}
)
add_dependencies(${LINKTRACK_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set(LINKTRACK_RVIZ_NAME "linktrack_rviz_converter")
add_executable(${LINKTRACK_RVIZ_NAME}
    src/linktrack/main_rviz_converter.cpp
)
target_link_libraries(${LINKTRACK_RVIZ_NAME}
    ${nutils}
)
add_dependencies(${LINKTRACK_RVIZ_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set(TOFSENSE_NAME "tofsense")
add_executable(${TOFSENSE_NAME}
    src/tofsense/init.cpp
    src/tofsense/main.cpp
)
target_link_libraries(${TOFSENSE_NAME}

    # ${catkin_LIBRARIES}
    ${nutils}
)
add_dependencies(${TOFSENSE_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set(EXE_NAME "tofsensem")
add_executable(${EXE_NAME}
    src/tofsensem/init.cpp
    src/tofsensem/main.cpp
)
target_link_libraries(${EXE_NAME}
    ${nutils}
)
add_dependencies(${EXE_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set(EXE_NAME "linktrack_aoa")
add_executable(${EXE_NAME}
    src/linktrack_aoa/init.cpp
    src/linktrack_aoa/main.cpp
    src/linktrack/protocols.cpp
)
target_link_libraries(${EXE_NAME}

    # ${catkin_LIBRARIES}
    ${nutils}
)
add_dependencies(${EXE_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set(EXE_NAME "iot")
add_executable(${EXE_NAME}
    src/iot/init.cpp
    src/iot/main.cpp
)
target_link_libraries(${EXE_NAME}
    ${nutils}
)
add_dependencies(${EXE_NAME} ${PROJECT_NAME}_generate_messages_cpp)

# ############
# # Testing ##
# ############
if(${CATKIN_ENABLE_TESTING})
    find_package(rostest REQUIRED)

    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    # # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}_test
        src/linktrack/init.cpp
        src/linktrack/protocols.cpp
        src/tofsense/init.cpp
        src/linktrack_aoa/init.cpp
        test/test_nlink_parser.cpp
        src/tofsensem/init.cpp
        src/iot/init.cpp
    )
endif()

if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${nutils})
endif()
