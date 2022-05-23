cmake_minimum_required(VERSION 3.10)

project(tricubic_interpolation)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif ()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(spdlog REQUIRED)

set(LIB_SPDLOG spdlog::spdlog)
set(LIB_EIGEN Eigen3::Eigen)

set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/bin")	#binaries will be copied to this directory
#message("BIN_DIR=" ${BIN_DIR})

find_package(Threads REQUIRED)

# libgbpcm library
add_library(libgbpcm STATIC
        src/lib/ioutils.cpp
        src/lib/ioutils.h
        src/lib/ptcloud.cpp
        src/lib/ptcloud.h
        src/lib/translationgrid.cpp
        src/lib/translationgrid.h
        src/lib/correspondences.cpp
        src/lib/correspondences.h
        src/lib/gbpcm_optimization.cpp
        src/lib/gbpcm_optimization.h
        src/lib/3rdparty/nanoflann/nanoflann.hpp)

target_link_libraries(libgbpcm PUBLIC ${LIB_SPDLOG} ${LIB_EIGEN})
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

# copy final executables to bin directory
file(MAKE_DIRECTORY ${BIN_DIR})
add_custom_command(TARGET gbpcm           POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gbpcm>           ${BIN_DIR})
add_custom_command(TARGET gbpcm-transform POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gbpcm-transform> ${BIN_DIR})
