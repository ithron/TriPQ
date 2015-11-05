cmake_minimum_required(VERSION 3.2)

project(TriPQ
  VERSION 0.1.0
  LANGUAGES CXX
)

add_library(TriPQ INTERFACE)
target_include_directories(TriPQ INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(TriPQ INTERFACE
  cxx_auto_type
  cxx_decltype
)

include(CMakePackageConfigHelpers)
install(
  DIRECTORY include/TriPQ
  DESTINATION include
)
install(
  TARGETS
    TriPQ
  EXPORT TriPQExport
  INCLUDES DESTINATION include
)
install(EXPORT TriPQExport DESTINATION lib/cmake/TriPQ)
export(EXPORT TriPQExport)

configure_package_config_file(TriPQConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/TriPQConfig.cmake
  INSTALL_DESTINATION lib/cmake/TriPQ
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/TriPQConfigVersion.cmake
  VERSION ${TriPQ_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/TriPQConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/TriPQConfigVersion.cmake
  DESTINATION lib/cmake/TriPQ
)

