cmake_minimum_required(VERSION 2.8)

set(POD_NAME OdometryCalibrator)

# CMAKE TWEAKS
#========================================================================
cmake_minimum_required (VERSION 2.8)
cmake_policy (VERSION 2.8)
cmake_policy (SET CMP0054 OLD)
cmake_policy (SET CMP0002 OLD) # allow duplicate target names (non-Makefile  generators, e.g., Xcode or VS, will bonk)
set (CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set (FIND_LIBRARY_USE_LIB64_PATHS 1)
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif ()

# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

find_package(PkgConfig REQUIRED)

# CONFIGURE C/C++
#========================================================================
set (CMAKE_C_CXX_COMMON_FLAGS "-Wall -Wfloat-equal -Wundef -Wno-strict-aliasing -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-macros -Wno-unknown-pragmas")
set (CMAKE_C_FLAGS "-u MAIN__ -std=gnu99 -fopenmp ${CMAKE_C_CXX_COMMON_FLAGS}")
set (CMAKE_CXX_FLAGS "-u MAIN__ -std=gnu++11 -fopenmp ${CMAKE_C_CXX_COMMON_FLAGS}")

## PREPARE IRAP LIBRARY
##========================================================================
#set (IRAP_LIB_DIR "${CMAKE_SOURCE_DIR}/third-party/build/irap/build/lib")
#set (IRAP_INC_DIR "${CMAKE_SOURCE_DIR}/third-party/build/irap/build/include")
#find_library (IRAP_LCMTYPES irap-lcmtypes PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_COMMON irap-common PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_COMMON++ irap-common++ PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_MATH irap-math PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_MATH++ irap-math++ PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_VISION irap-vision PATHS ${IRAP_LIB_DIR})
#find_library (IRAP_VISION++ irap-vision++ PATHS ${IRAP_LIB_DIR})
#include_directories(${IRAP_INC_DIR})


# CONFIG FILES
#========================================================================
set (CONFIG_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)

add_custom_target (config ALL
  COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/build/share
  COMMAND ln -sf ${CONFIG_SOURCE_PATH} ${CMAKE_SOURCE_DIR}/build/share/.
)

install (DIRECTORY ${CONFIG_SOURCE_PATH}
  DESTINATION ${CMAKE_SOURCE_DIR}/build/share/.
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ WORLD_WRITE
  DIRECTORY_PERMISSIONS  OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE  GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE
  PATTERN ".svn" EXCLUDE
  )

# Gen and cook lcmtypes
#========================================================================
include(${PROJECT_SOURCE_DIR}/lcmdefs/PACKAGES.cmake)
lcmtypes_build()


set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
set(CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)
set(CMAKE_EXTRA_GENERATOR "Sublime Text 2" CACHE INTERNAL "" FORCE)
# INCLUDE PROJECT SUBDIRECTORY
#========================================================================
add_subdirectory(src)
#add_subdirectory(config)
