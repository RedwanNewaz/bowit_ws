include(FetchContent) # once in the project to include the module
include(ExternalProject)

FetchContent_Declare(limbo
        URL "https://github.com/resibots/limbo/archive/refs/tags/v2.1.0.zip"
)
FetchContent_MakeAvailable(limbo)

ExternalProject_Add(
        "nlopt"
        URL "https://github.com/stevengj/nlopt/archive/refs/tags/v2.7.1.zip"
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)

ExternalProject_Add(
        "libcmaes"
        URL "https://github.com/CMA-ES/libcmaes/archive/refs/tags/v0.10.zip"
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem program_options REQUIRED)

link_directories(${CMAKE_BINARY_DIR}/install/lib)

set(limbo_INCLUDE_DIRS
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/install/include
        ${CMAKE_BINARY_DIR}/_deps/limbo-src/src)

set(limbo_LIBRARIES
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    -lnlopt -lm
    libcmaes.so
)

