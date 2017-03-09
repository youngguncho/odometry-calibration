macro (libhandler_bot2)
  set (INSTALLMSG "did you build libbot2 in third-party?")
  libhandler_find_library (bot2-core ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-frames ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-frames-renderers ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-lcmgl-client ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-lcmgl-renderer ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-param-client ${INSTALLMSG} ${ARGN})
  libhandler_find_library (bot2-vis ${INSTALLMSG} ${ARGN})
  if (BOT2-CORE_FOUND AND
      BOT2-FRAMES_FOUND AND
      BOT2-FRAMES-RENDERERS_FOUND AND
      BOT2-LCMGL-CLIENT_FOUND AND
      BOT2-LCMGL-RENDERER_FOUND AND
      BOT2-PARAM-CLIENT_FOUND AND
      BOT2-VIS_FOUND
      )
    set (IRPLIB_BOT2 
      ${BOT2-CORE_LIBRARIES}
      ${BOT2-FRAMES_LIBRARIES}
      ${BOT2-FRAMES-RENDERERS_LIBRARIES}
      ${BOT2-LCMGL-CLIENT_LIBRARIES}
      ${BOT2-LCMGL-RENDERER_LIBRARIES}
      ${BOT2-PARAM-CLIENT_LIBRARIES}
      ${BOT2-VIS_LIBRARIES}
      )
  else ()
    libhandler_error (${INSTALLMSG})
  endif ()

  # to link against bot2, you also need to link against glib, so be
  # helpful and include it here
  libhandler_glib ()
  if (GLIB_FOUND)
    set (IRPLIB_BOT2 ${IRPLIB_BOT2} ${IRPLIB_GLIB})
  endif ()
endmacro ()
