include(ExternalProject)


ExternalProject_Add(
    "libccd"
    URL "https://github.com/danfis/libccd/archive/refs/tags/v2.1.zip"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)
ExternalProject_Add(
    "fcl"
    URL "https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.zip"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
        -DFCL_BUILD_TESTS=OFF
)

ExternalProject_Add(
        "octomap"
        URL "https://github.com/OctoMap/octomap/archive/refs/tags/v1.9.6.zip"
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)

link_directories(${CMAKE_BINARY_DIR}/install/lib)

set(fcl_INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}/install/include
)

find_package(Eigen3 REQUIRED)
set(fcl_LIBRARIES
    Eigen3::Eigen
    libccd.so
    libfcl.so
    liboctomap.so
)