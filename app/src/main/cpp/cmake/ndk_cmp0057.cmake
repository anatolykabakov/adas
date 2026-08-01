# Injected via tools.cmake.cmaketoolchain:user_toolchain
# (must run before NDK toolchain).
# NDK r27 flags.cmake uses IN_LIST; packages with
# cmake_minimum_required(<3.3) leave CMP0057 unset.
set(CMAKE_POLICY_DEFAULT_CMP0057 "NEW" CACHE STRING
    "Enable IN_LIST for NDK flags.cmake" FORCE
)
