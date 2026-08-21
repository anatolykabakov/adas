find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

add_executable(
  adas_tests
  main.cpp
  test_middleware.cpp
  test_zmq_imu.cpp
  test_utils.cpp
  test_lat_control.cpp
  test_pp_planner.cpp
  test_lane_keep_gates.cpp
  test_panda_mqb.cpp
  test_planner.cpp
  test_safety_warn.cpp
  test_long_plan.cpp
  test_vehicle_ekf.cpp
  test_speed_filter.cpp
  test_road_roll.cpp
  test_params_learner.cpp
)

target_compile_definitions(
  adas_tests
  PRIVATE ADAS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)

target_include_directories(
  adas_tests
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..
          ${CMAKE_CURRENT_BINARY_DIR}/..
)

set(_adas_shipped_config "${CMAKE_CURRENT_SOURCE_DIR}/../../assets/config.json")
set(_adas_test_map "${CMAKE_CURRENT_SOURCE_DIR}/data/test_road.admap")
target_compile_definitions(
  adas_tests
  PRIVATE
    GTEST_HAS_TR1_TUPLE=0 ADAS_SHIPPED_CONFIG_JSON="${_adas_shipped_config}"
    ADAS_TEST_MAP_FILE="${_adas_test_map}"
)

target_link_libraries(
  adas_tests
  PRIVATE
    adas_app_common
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
    Threads::Threads
    rt
)

add_test(
  NAME adas_tests
  COMMAND adas_tests
)
set_tests_properties(
  adas_tests
  PROPERTIES TIMEOUT 60
)
