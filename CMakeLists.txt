cmake_minimum_required(VERSION 3.10)

project(tricubic_interpolation)

set(CMAKE_CXX_STANDARD 17)

# Add compiler warnings for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    add_compile_options(
        -Wall        # Enable most warning flags
        -Wextra      # Enable extra warning flags
        -Wpedantic   # Enable pedantic mode
        -Werror      # Treat warnings as errors
    )
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(spdlog REQUIRED)
find_package(nanoflann)
find_package(PDAL 2.3.0 REQUIRED)
find_package(cxxopts 3.0.0 REQUIRED)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

set(LIB_SPDLOG spdlog::spdlog)
set(LIB_EIGEN Eigen3::Eigen)
set(LIB_NANOFLANN nanoflann::nanoflann)

set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/bin")	#binaries will be copied to this directory
#message("BIN_DIR=" ${BIN_DIR})

find_package(Threads REQUIRED)

# libgbpcm library
add_library(libgbpcm STATIC
    src/lib/io_utils.cpp
    src/lib/io_utils.hpp
    src/lib/pt_cloud.cpp
    src/lib/pt_cloud.hpp
    src/lib/translation_grid.cpp
    src/lib/translation_grid.hpp
    src/lib/correspondences.cpp
    src/lib/correspondences.hpp
    src/lib/gbpcm_optimization.cpp
    src/lib/gbpcm_optimization.hpp)

target_link_libraries(libgbpcm PUBLIC ${LIB_SPDLOG} ${LIB_EIGEN} ${LIB_NANOFLANN} ${PDAL_LIBRARIES})
target_include_directories(libgbpcm PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/lib)
target_include_directories(libgbpcm PRIVATE ${CMAKE_CURRENT_LIST_DIR})

set_target_properties(libgbpcm PROPERTIES DEBUG_POSTFIX _d)

# gbpcm executable
add_executable(gbpcm src/prog/gbpcm.cpp)
target_link_libraries(gbpcm libgbpcm)
target_include_directories(gbpcm PRIVATE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(gbpcm PROPERTIES DEBUG_POSTFIX _d)

# gbpcm-transform executable
add_executable(gbpcm-transform src/prog/gbpcm_transform.cpp)
target_link_libraries(gbpcm-transform libgbpcm)
target_include_directories(gbpcm-transform PRIVATE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(gbpcm-transform PROPERTIES DEBUG_POSTFIX _d)

# Copy final executables to bin directory
file(MAKE_DIRECTORY ${BIN_DIR})
add_custom_command(TARGET gbpcm           POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gbpcm>           ${BIN_DIR})
add_custom_command(TARGET gbpcm-transform POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gbpcm-transform> ${BIN_DIR})

# Enable testing
enable_testing()

# Add tests for the shell scripts
add_test(
    NAME gbpcm.EagleScript
    COMMAND /bin/bash ${CMAKE_SOURCE_DIR}/test/test-eagle.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)
add_test(
    NAME gbpcm.NordbahnScript
    COMMAND /bin/bash ${CMAKE_SOURCE_DIR}/test/test-nordbahn.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)
