set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3)
find_package(Sophus)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/camera CAMERA_SRC_FILES)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/veta HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/veta/camera CAMERA_HEADER_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${SRC_FILES} ${CAMERA_SRC_FILES}
        ${HEADER_FILES} ${CAMERA_HEADER_FILES}
)

set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        # only when building from the source tree
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        # only when using the lib from the install path
        $<INSTALL_INTERFACE:include>
        ${ADDITIONAL_INCLUDE_DIRS}
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        Sophus::Sophus
)

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE
        ${LIBRARY_NAME}
)