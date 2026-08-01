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
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pyadas"
)

# Copy next to bag/host scripts (app/src/main/scripts/pyadas).
set(SCRIPTS_PYADAS "${PROJECT_SOURCE_DIR}/../scripts/pyadas")
add_custom_command(
  TARGET core
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${SCRIPTS_PYADAS}"
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:core> "${SCRIPTS_PYADAS}/"
  COMMAND
    ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/pyadas/__init__.py"
    "${SCRIPTS_PYADAS}/__init__.py"
  COMMENT "Copy pyadas core module into scripts/"
  VERBATIM
)

install(
  TARGETS core
  COMPONENT python
  LIBRARY DESTINATION pyadas
)
