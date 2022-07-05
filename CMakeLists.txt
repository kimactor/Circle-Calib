cmake_minimum_required(VERSION 3.23 FATAL_ERROR)
project(calibration)
add_compile_options(-fPIC)

set(CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")
set(CMAKE_BUILD_TYPE "Debug")
set(PACKAGE_VERSION 1.0)

IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
ENDIF()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp 0.5.2 REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${YAML_CPP_INCLUDE_DIR})

aux_source_directory(./src DIR_SRC)
add_executable(my_test ${DIR_SRC})
target_link_libraries(my_test Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp)


##--------------------- Install and Package target -------------------------##
# Install
install(TARGETS my_test
        EXPORT my_test
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
# PUBLIC_HEADER DESTINATION include
file(GLOB_RECURSE PYTHON3 ${PROJECT_SOURCE_DIR}/python3/*.py)
install(FILES ${PYTHON3} DESTINATION python3)

install(
  EXPORT my_test
  FILE my_testTargets.cmake
  DESTINATION lib/cmake/my_test)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    my_testConfig.cmake.in ${PROJECT_BINARY_DIR}/my_testConfig.cmake
    INSTALL_DESTINATION lib/cmake/my_test)

write_basic_package_version_file(
    my_testConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES "${PROJECT_BINARY_DIR}/my_testConfig.cmake"
              "${PROJECT_BINARY_DIR}/my_testConfigVersion.cmake"
        DESTINATION lib/cmake/my_test)

# Package, These variables should set before including CPack module
set(CPACK_GENERATOR "ZIP")
set(CPACK_SET_DESTDIR ON)  # 支持指定安装目录
set(CPACK_INSTALL_PREFIX "Calibration")
include(CPack)

