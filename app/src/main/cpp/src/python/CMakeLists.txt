find_package(pybind11 CONFIG REQUIRED)
find_package(
  Python3
  COMPONENTS
    Interpreter
    Development
    REQUIRED
)

pybind11_add_module(core core.cpp)
target_link_libraries(
  core
  PRIVATE adas_app_common
)
target_include_directories(
  core
  PRIVATE ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_BINARY_DIR}
)

set_target_properties(
  core
  PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pyadas"
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)
target_link_options(core PRIVATE "-Wl,--disable-new-dtags")

file(
  GLOB ACADOS_RUNTIME_LIBS "${acados_PACKAGE_FOLDER_RELEASE}/lib/*.so"
  "${acados_PACKAGE_FOLDER_DEBUG}/lib/*.so"
)

get_filename_component(REPO_ROOT "${PROJECT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(SCRIPTS_PYADAS "${REPO_ROOT}/scripts/pyadas")
add_custom_command(
  TARGET core
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${SCRIPTS_PYADAS}"
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:core> "${SCRIPTS_PYADAS}/"
  COMMAND
    ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/pyadas/__init__.py"
    "${SCRIPTS_PYADAS}/__init__.py"
  COMMAND
    ${CMAKE_COMMAND} -E copy_if_different ${ACADOS_RUNTIME_LIBS}
    "${SCRIPTS_PYADAS}/"
  COMMENT "Copy pyadas core module into scripts/"
  VERBATIM
)

install(
  TARGETS core
  COMPONENT python
  LIBRARY DESTINATION pyadas
)
