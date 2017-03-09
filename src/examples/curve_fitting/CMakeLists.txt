cmake_minimum_required (VERSION 2.8.12)

project (curve_fitting)

set (SRC
  curve_fitting.cpp
)

# link other 3rd party libraries
libhandler_lcm ()

# find ceres libraries
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (curve_fitting ${SRC})
target_link_libraries (curve_fitting
    ${IRPLIB_LCM}
    ${CERES_LIBRARIES}
)

pods_install_executables (curve_fitting)
