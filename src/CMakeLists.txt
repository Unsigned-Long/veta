set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3)
find_package(Sophus)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/camera CAMERA_SRC_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${SRC_FILES}
        ${CAMERA_SRC_FILES}
)

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        ${EIGEN_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
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