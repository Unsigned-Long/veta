cmake_minimum_required(VERSION 3.10)

project(veta VERSION 1.0)

# -----------
# set lib name ans space
# -----------
set(LIBRARY_NAME veta)

set(LIBRARY_NAMESPACE Veta)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# -----------
# for install
# -----------

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Export package for use from the build tree
set(LIBRARY_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/${LIBRARY_NAME}/cmake)

set_target_properties(${LIBRARY_NAME} PROPERTIES EXPORT_NAME ${LIBRARY_NAMESPACE})

export(
        TARGETS ${LIBRARY_NAME}
        NAMESPACE ${LIBRARY_NAMESPACE}::
        FILE ${LIBRARY_NAMESPACE}Targets.cmake
)
export(
        PACKAGE ${LIBRARY_NAMESPACE}
)

configure_package_config_file(
        ${LIBRARY_NAMESPACE}Config.cmake.in
        ${CMAKE_BINARY_DIR}/${LIBRARY_NAMESPACE}Config.cmake
        INSTALL_DESTINATION ${LIBRARY_CMAKE_EXPORT_DIR}
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Write version to file
write_basic_package_version_file(
        ${LIBRARY_NAMESPACE}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)