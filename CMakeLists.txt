cmake_minimum_required(VERSION 3.10)

project(veta VERSION 1.0)

# -----------
# set lib name ans space
# -----------
set(LIBRARY_NAME veta)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# -----------
# for install
# -----------

# Configuration
set(CONFIG_INSTALL_DIR "lib/cmake/${LIBRARY_NAME}")
set(INCLUDE_INSTALL_DIR "include")
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}ConfigVersion.cmake")
set(PROJ_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${LIBRARY_NAME}Targets")

# Include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Note: PROJECT_VERSION is used as a VERSION
write_basic_package_version_file("${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion)

# Configure '<PROJECT-NAME>Config.cmake'
# Use variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
        "${PROJ_CONFIG}"
        INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Export '<LIBRARY_NAME>Targets.cmake' to build dir (to find package in build dir without install)
export(
        TARGETS ${LIBRARY_NAME}
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_EXPORT_NAME}.cmake"
)

# Targets:
install(
        TARGETS ${LIBRARY_NAME}
        EXPORT "${TARGETS_EXPORT_NAME}"
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)

# Config
install(
        FILES "${PROJ_CONFIG}" "${VERSION_CONFIG}"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Config
install(
        EXPORT "${TARGETS_EXPORT_NAME}"
        NAMESPACE "${namespace}"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Headers
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/include/${LIBRARY_NAME}
        DESTINATION ${INCLUDE_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN "*.hpp"
)
