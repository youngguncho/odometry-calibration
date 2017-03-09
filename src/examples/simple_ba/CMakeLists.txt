cmake_minimum_required (VERSION 2.8.12)

project (simple_ba)

set (SRC
  simple_ba.cpp
)

# link other 3rd party libraries
libhandler_lcm ()

# find ceres libraries
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (simple_ba ${SRC})
target_link_libraries (simple_ba
    ${IRPLIB_LCM}
    ${CERES_LIBRARIES}
)

pods_install_executables (simple_ba)
