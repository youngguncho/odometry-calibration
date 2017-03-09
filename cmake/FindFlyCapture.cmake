# Find FlyCapture2 on Windows/Linux.
#
# Once loaded this will define
#   FLYCAPTURE_FOUND           - system has FlyCapture2
#   FLYCAPTURE_INCLUDE_DIR     - include directory for FlyCapture2
#   FLYCAPTURE_LIBRARY_DIRS    - library directries for FlyCapture2
#   FLYCAPTURE2_LIB            - FlyCapture2.lib you need to link to
#   FLYCAPTURE2_DLL            - FlyCapture2.dll you need to link to

# Modified by Ayoung Kim 2015.06.29 to accommodate more for linux
# tested on ubuntu 14.04

if(MSVC)
  if(CMAKE_CL_64)

    if(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
      set(FLYCAPTURE_ROOT "C:\\Program Files\\Point Grey Research\\FlyCapture2")

    else(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")

      if(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
        set(FLYCAPTURE_ROOT "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
      endif(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
    endif(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")

  else(CMAKE_CL_64)

    if(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")
      set(FLYCAPTURE_ROOT "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

    else(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

      if(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
        set(FLYCAPTURE_ROOT "C:\\Program Files\\Point Grey Research\\FlyCapture2")
      endif(EXISTS "C:\\Program Files\\Point Grey Research\\FlyCapture2")
    endif(EXISTS "C:\\Program Files (x86)\\Point Grey Research\\FlyCapture2")

  endif(CMAKE_CL_64)
  message(STATUS "FLYCAPTURE_ROOT : ${FLYCAPTURE_ROOT}")

  if(FLYCAPTURE_ROOT)
    set(FLYCAPTURE_INCLUDE_DIR "${FLYCAPTURE_ROOT}/include")
    message(STATUS "FLYCAPTURE_INCLUDE_DIR : ${FLYCAPTURE_INCLUDE_DIR}")
  endif(FLYCAPTURE_ROOT)

  if(FLYCAPTURE_INCLUDE_DIR)
    if(CMAKE_CL_64)
      if(EXISTS "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin64")

      else(EXISTS "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin")
  	  endif(EXISTS "${FLYCAPTURE_ROOT}/lib64")

    else(CMAKE_CL_64)
      if(EXISTS "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin")

      else(EXISTS "${FLYCAPTURE_ROOT}/lib")
        set(FLYCAPTURE_LIBRARY_DIR "${FLYCAPTURE_ROOT}/lib64")
        set(FLYCAPTURE_DLL_DIR "${FLYCAPTURE_ROOT}/bin64")
  	  endif(EXISTS "${FLYCAPTURE_ROOT}/lib")

    endif(CMAKE_CL_64)

    find_library(FLYCAPTURE2_LIB
      NAMES FlyCapture2
      PATHS
	    "${FLYCAPTURE_LIBRARY_DIR}"
    )
    find_library(FLYCAPTURE2_DLL
      NAMES FlyCapture2
      PATHS
	    "${FLYCAPTURE_DLL_DIR}"
    )
    set(FLYCAPTURE2_LIBS ${FLYCAPTURE2_LIB})
  endif(FLYCAPTURE_INCLUDE_DIR)

else(MSVC)

  find_path(FLYCAPTURE_INCLUDE_DIR
    NAMES FlyCapture2.h
    PATHS "/usr/include/flycapture"
  )

  if(FLYCAPTURE_INCLUDE_DIR)
    set(FLYCAPTURE_LIBRARY_DIR /usr/lib/)
    find_library(FLYCAPTURE2_LIB NAMES flycapture PATHS "${FLYCAPTURE_LIBRARY_DIR}")    
    find_library(FLYCAPTURE2C_LIB NAMES flycapture-c PATHS "${FLYCAPTURE_LIBRARY_DIR}")    
    find_library(FLYCAPTURE2GUI_LIB NAMES flycapturegui PATHS "${FLYCAPTURE_LIBRARY_DIR}")    
    find_library(FLYCAPTURE2GUIC_LIB NAMES flycapturegui-c PATHS "${FLYCAPTURE_LIBRARY_DIR}")    

    set(FLYCAPTURE2_LIBS ${FLYCAPTURE2_LIB} ${FLYCAPTURE2C_LIB} ${FLYCAPTURE2GUI_LIB} ${FLYCAPTURE2GUIC_LIB})
  endif(FLYCAPTURE_INCLUDE_DIR)
endif(MSVC)

message(STATUS "FLYCAPTURE_LIBRARY_DIR : ${FLYCAPTURE_LIBRARY_DIR}")
message(STATUS "FLYCAPTURE2_LIBS : ${FLYCAPTURE2_LIBS}")

if(FLYCAPTURE2_LIBS)
  set(FLYCAPTURE_FOUND TRUE)
else(FLYCAPTURE2_LIBS)
  message(STATUS "Warning: cannot find FlyCapture2")
endif(FLYCAPTURE2_LIBS)


