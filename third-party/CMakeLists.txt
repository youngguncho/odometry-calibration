project (third-party)

# CMAKE TWEAKS
#========================================================================
cmake_minimum_required (VERSION 2.8)
cmake_policy (VERSION 2.8)
set (CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set (FIND_LIBRARY_USE_LIB64_PATHS 1)
set (THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif ()

# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake/")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

# CUSTOM TARGETS
#========================================================================
add_custom_target (third-party ALL
  COMMAND cmake ..
  )
add_custom_target (uninstall
  COMMAND cmake ..
  )
add_custom_target (cleanup
  COMMAND cmake ..
  )

# ADD THIRD-PARTY LIBRARY TARGETS BELOW IN ALPHABETICAL ORDER
#========================================================================
#include (irap.cmake)
include (lcm.cmake)
#include (libbot2.cmake)

