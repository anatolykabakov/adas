cmake_minimum_required(VERSION 3.17)
project(adas_app LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_FOR_ANDROID "Build Android shared library" ON)
option(BUILD_TESTING "Build unit tests" OFF)
option(BUILD_PYTHON_BINDINGS "Build pyadas pybind11 module (desktop)" OFF)

find_package(Protobuf REQUIRED)
find_package(cppzmq REQUIRED)
find_package(libusb REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(acados REQUIRED)

file(
  GLOB PROTO_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/../proto/*.proto
)
# cmake-lint: disable=C0103
# FindProtobuf expects this name for --proto_path.
set(Protobuf_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../proto)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(protobuf_generated STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(
  protobuf_generated
  PUBLIC protobuf::libprotobuf
)
target_include_directories(
  protobuf_generated
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)

add_subdirectory(src)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()
