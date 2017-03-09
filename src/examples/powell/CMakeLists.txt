cmake_minimum_required (VERSION 2.8.12)

project (powell)

set (SRC
  powell.cpp
)

# link other 3rd party libraries
libhandler_lcm ()

# find ceres libraries
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (powell ${SRC})
target_link_libraries (powell
    ${IRPLIB_LCM}
    ${CERES_LIBRARIES}
)

pods_install_executables (powell)
