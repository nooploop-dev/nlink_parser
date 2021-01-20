cmake_minimum_required(VERSION 3.0.0)
project(nlink_unpack VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(nlink_unpack 
example.c
nlink_utils.c
nlink_linktrack_nodeframe0.c
nlink_linktrack_nodeframe1.c
nlink_tofsense_frame0.c
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
