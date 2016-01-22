cmake_minimum_required(VERSION 3.2)

project(TriPQ
  VERSION 0.1.0
  LANGUAGES CXX
)

option(DEBUG_LOOP_DETECTION "Loop detection in debug build?" OFF)
if (DEBUG_LOOP_DETECTION)
  option(DEBUG_THROW_ON_LOOP
    "Throw a LoopException instead of assert when detecting a loop?" OFF
  )
endif()

add_library(TriPQ INTERFACE)
target_include_directories(TriPQ INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(DEBUG_LOOP_DETECTION)
  if (DEBUG_THROW_ON_LOOP)
    target_compile_definitions(TriPQ INTERFACE
      TriPQLoopDetection TriPQThrowOnLoop
    )
  else()
    target_compile_definitions(TriPQ INTERFACE TriPQLoopDetection)
  endif()
endif()

if(MAXIMUM_HOP_LIMIT)
  target_compile_definitions(TriPQ INTERFACE TriPQHopLimit=${MAXIMUM_HOP_LIMIT})
endif()

target_compile_features(TriPQ INTERFACE
  cxx_auto_type
  cxx_decltype
  cxx_decltype_auto
  cxx_return_type_deduction
  cxx_default_function_template_args
  cxx_right_angle_brackets
  cxx_rvalue_references
  cxx_template_template_parameters
)

option(BUILD_EXAMPLES "Build examples" OFF)

if (BUILD_EXAMPLES)
  find_package(CGAL QUIET)
  if (CGAL_USE_FILE)
    include(${CGAL_USE_FILE})
    add_library(CGAL INTERFACE IMPORTED)
    set_property(TARGET CGAL
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CGAL_INCLUDE_DIRS}
    )
    set_property(TARGET CGAL
      PROPERTY INTERFACE_LINK_LIBRARIES ${CGAL_LIBRARIES}
    )
  endif()

  if(TARGET CGAL)
    add_executable(ExampleSphericalTriangulation
      examples/exampleSphericalTriangulation.cpp
    )
    target_link_libraries(ExampleSphericalTriangulation PRIVATE TriPQ CGAL)
  else()
    message(STATUS
      "ExampleSphericalTriangulation disabled because CGAL was not found")
  endif()

endif()

################################
# Documentation
#

find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxy.config.in
    ${CMAKE_CURRENT_BINARY_DIR}/doxy.config @ONLY
  )
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxy.config
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
endif()

################################
# Installation and Packaging
#

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

