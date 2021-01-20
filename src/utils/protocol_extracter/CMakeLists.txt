cmake_minimum_required(VERSION 3.0.0)
project(nlink_protocol VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(nlink_protocol 
example.cpp
nprotocol_base.cpp
nprotocol_extracter.cpp
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
