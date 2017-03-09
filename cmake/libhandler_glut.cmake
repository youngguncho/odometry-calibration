macro (libhandler_glut)
  # freeglut requires linking against -lXi and -lXmu, so check for those
  libhandler_find_library (Xi "on ubuntu `sudo apt-get install libxi-dev`")
  libhandler_find_library (Xmu "on ubuntu `sudo apt-get install libxmu-dev`")

  libhandler_find_package (GLUT "on ubuntu `sudo apt-get install freeglut3-dev`" ${ARGN})
  if (GLUT_FOUND)
    include_directories (${GLUT_INCLUDE_DIR})
    set (IRPLIB_GLUT ${GLUT_LIBRARIES})
  endif ()
endmacro ()
