add_library(adas_panda STATIC panda.cc panda_comms.cc)

target_include_directories(
  adas_panda
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(
  adas_panda
  PUBLIC libusb::libusb
)

if(BUILD_FOR_ANDROID)
  target_compile_definitions(
    adas_panda
    PUBLIC LIBUSB_OPTION_WEAK_AUTHORITY BUILD_FOR_ANDROID
  )
  target_link_libraries(
    adas_panda
    PUBLIC log
           android
  )
endif()
