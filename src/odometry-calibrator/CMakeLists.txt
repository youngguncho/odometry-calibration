cmake_minimum_required (VERSION 2.8.12)

set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

project (odom-calibrator)

# Find includes in corresponding build directories
# Adds in each directory ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_BINARY_DIR}
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the Qt libraries
find_package (Qt5Widgets)
find_package (Qt5Gui)
find_package (Qt5OpenGL)

set (SRC
  main.cpp 
  MainWindow.cpp MainWindow.h
  transform.cpp   transform.h
  odomproblem.cpp odomproblem.h
)

# add UI file
qt5_wrap_ui (calibrator_ui_moc MainWindow.ui)

# add Resources
qt5_add_resources(SHADER_RSC_ADDED resources.qrc)

# link other 3rd party libraries
libhandler_lcm ()
libhandler_opengl ()
libhandler_glut ()
libhandler_eigen3 ()

# find ceres libraries
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (odom-calibrator ${SRC} ${SHADER_RSC_ADDED} ${calibrator_ui_moc})
target_link_libraries (odom-calibrator
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
    ${IRPLIB_LCM}
    ${IRPLIB_GLUT}
    ${IRPLIB_OPENGL}
    ${IRPLIB_EIGEN3}
    ${CERES_LIBRARIES}
)

pods_install_executables (odom-calibrator)
