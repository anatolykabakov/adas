add_subdirectory(panda)
add_subdirectory(utils)

add_library(
  adas_app_common
  STATIC
  adas_app.cpp
  services/zmq_bridge.cpp
  services/panda.cpp
  services/lane_keep.cpp
  services/long_plan.cpp
  services/safety_warn.cpp
  services/traffic_sign.cpp
  services/localization.cpp
  services/camera_calib.cpp
  services/topic_convert.cpp
  services/imu_calib.cpp
  services/internal_subscriber.cpp
  services/middleware_stats.cpp
  platform/volkswagen/mqbcan.cpp
  platform/volkswagen/carcontroller.cpp
  platform/volkswagen/mqb_car_state_decoder.cpp
  platform/volkswagen/panda_safety_supervisor.cpp
  services/map_data.cpp
  mapmatch/track.cpp
  mapmatch/road_map.cpp
  mapmatch/road_route.cpp
  mapmatch/fit.cpp
  mapmatch/search.cpp
  mapmatch/window_search.cpp
  utils/protobuf_utils.cpp
  utils/can_parser.cc
  lateral/pp.cpp
  utils/path_lateral_state.cpp
  lateral/visionpilot_mpc.cpp
  lateral/flowpilot_mpc.cpp
  utils/vehicle_ekf.cpp
  utils/online_localizer.cpp
  utils/vanishing_point_calib.cpp
  utils/pose_calibrator.cpp
  utils/topic_convert.cpp
  utils/imu_calibrator.cpp
  utils/adas_config.cpp
  utils/proto_convert.cpp
)
target_link_libraries(
  adas_app_common
  PUBLIC
    cppzmq
    protobuf_generated
    adas_panda
    JsonCpp::JsonCpp
    Eigen3::Eigen
)
target_include_directories(
  adas_app_common
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

if(BUILD_FOR_ANDROID)
  target_compile_definitions(
    adas_app_common
    PUBLIC BUILD_FOR_ANDROID
  )
  target_link_libraries(
    adas_app_common
    PUBLIC log
           android
  )
  add_library(adas_app_android SHARED adas_app_android.cpp utils/yuv_warp.cpp)
  target_link_libraries(
    adas_app_android
    PUBLIC
      adas_app_common
      adas_panda
      android
      log
  )
  target_compile_definitions(
    adas_app_android
    PUBLIC BUILD_FOR_ANDROID
  )
  target_compile_options(
    adas_app_android
    PRIVATE -O3
  )

  add_library(thneedrunner SHARED thneed/thneed_runner.cpp thneed/json11.cpp)
  target_include_directories(
    thneedrunner
    PRIVATE ${PROJECT_SOURCE_DIR}/include
  )
  target_link_libraries(
    thneedrunner
    PRIVATE
      dl
      log
      android
  )
  target_compile_options(
    thneedrunner
    PRIVATE
      -O3
      -Wno-unused-variable
      -Wno-unused-function
  )
else()
  add_executable(
    adas_app_linux
    adas_app_linux.cpp
  )
  target_link_libraries(
    adas_app_linux
    PUBLIC
      adas_app_common
      adas_panda
      pthread
      rt
  )
endif()

option(BUILD_PYTHON_BINDINGS "Build pyadas pybind11 module for sim/vis" OFF)
if(BUILD_PYTHON_BINDINGS
   AND NOT BUILD_FOR_ANDROID
)
  add_subdirectory(python)
endif()
