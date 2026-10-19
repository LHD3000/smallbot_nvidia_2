cmake_minimum_required(VERSION 3.19)
project(control-algorithm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(control_algorithm SHARED
    src/evalute/dynamic.cpp)

target_include_directories(control_algorithm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# find_package(osqp REQUIRED)
# target_link_libraries(control_algorithm PUBLIC osqp::osqp)
install(
    DIRECTORY include/
    DESTINATION include)
install(
    TARGETS control_algorithm
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib)
install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    PATH_VARS PROJECT_NAME)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "0.1.0"
    COMPATIBILITY AnyNewerVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION "0.1.0")
set(CPACK_PACKAGE_VENDOR "CZQ")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "pid and filter control algorithm")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "czq 1448973286@qq.com")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")

include(CPack)