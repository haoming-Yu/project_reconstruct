cmake_minimum_required(VERSION 3.10)

# Project name
project(geo_detector LANGUAGES CXX CUDA)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find pthread lib in Linux
find_package(Threads REQUIRED)

# ------------------------------------------------
#                 Dependencies
# ------------------------------------------------
## CUDA
find_package(CUDA 10.0 REQUIRED)
IF (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    # Optional: Specify the arch of your CUDA hardware here
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -gencode arch=compute_52,code=sm_52 --expt-relaxed-constexpr -Wno-deprecated-declarations) # -Wno-deprecated-declarations
ENDIF ()

## OpenCV
# Optional: Set OpenCV_DIR if you want to use a custom version of OpenCV
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include/opencv4/opencv2/")
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIB_DIR})
else ()
    message(FATAL_ERROR "OpenCV not found!")
endif ()

## Eigen3
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif (EIGEN3_INCLUDE_DIR)
# ------------------------------------------------

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-deprecated-declarations") # -Weffc++

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# Add include directories
include_directories(
    ${PROJECT_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/cuda/include
)

# Add source files
file(GLOB_RECURSE SOURCES
    ${PROJECT_SOURCE_DIR}/cuda/*.cu
    ${PROJECT_SOURCE_DIR}/*.cpp
)

# cuda_add_library(${PROJECT_NAME} STATIC ${SOURCES})
add_executable(${PROJECT_NAME} ${SOURCES})

target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-function -Wno-unused-variable>
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr -Wno-deprecated-declarations>
)

# Link libraries
set(CAM_LIB_PATH ${CMAKE_SOURCE_DIR}/lib/oToCAM_lib.a)

target_link_libraries(${PROJECT_NAME}
    Threads::Threads
    ${CAM_LIB_PATH}
	${OpenCV_LIBS}
)