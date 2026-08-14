add_subdirectory(panda)

set(ACADOS_OCP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lateral/acados_lat_ocp)
file(
  GLOB ACADOS_OCP_SRC ${ACADOS_OCP_DIR}/acados_solver_lat.c
  ${ACADOS_OCP_DIR}/lat_model/*.c ${ACADOS_OCP_DIR}/lat_cost/*.c
)
add_library(adas_acados_lat STATIC ${ACADOS_OCP_SRC})
set_target_properties(
  adas_acados_lat
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             C_STANDARD 99
)
target_include_directories(
  adas_acados_lat
  PUBLIC ${ACADOS_OCP_DIR}
)
target_link_libraries(
  adas_acados_lat
  PUBLIC acados::acados
)

add_library(
  adas_app_common
  STATIC
  adas_app.cpp
  services/zmq_bridge.cpp
  services/platform.cpp
  services/control.cpp
  services/planner.cpp
  utils/lane_keep_gates.cpp
  services/safety_warn.cpp
  services/traffic_sign.cpp
  services/localization.cpp
  services/camera_calib.cpp
  services/internal_subscriber.cpp
  services/middleware_stats.cpp
  platform/volkswagen/mqbcan.cpp
  platform/volkswagen/carcontroller.cpp
  platform/volkswagen/car_iface.cpp
  platform/volkswagen/mqb_car_state_decoder.cpp
  platform/volkswagen/panda_safety_supervisor.cpp
  services/map_data.cpp
  mapmatch/track.cpp
  mapmatch/road_map.cpp
  mapmatch/road_route.cpp
  mapmatch/fit.cpp
  mapmatch/search.cpp
  mapmatch/window_search.cpp
  utils/can_parser.cc
  utils/path_lateral_state.cpp
  lateral/pp_planner.cpp
  lateral/vp_planner.cpp
  lateral/fp_planner.cpp
  lateral/kappa_solver.cpp
  lateral/visionpilot_mpc.cpp
  lateral/flowpilot_mpc.cpp
  utils/vehicle_ekf.cpp
  utils/online_localizer.cpp
  utils/vanishing_point_calib.cpp
  utils/pose_calibrator.cpp
  utils/imu_calibrator.cpp
  utils/adas_config.cpp
  utils/lane_path.cpp
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

target_sources(
  adas_app_common
  PRIVATE lateral/acados_lat_mpc.cpp
)
target_link_libraries(
  adas_app_common
  PUBLIC adas_acados_lat
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
