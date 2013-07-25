remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})

remake_add_library(dsp3000 PREFIX OFF
  LINK ${CMAKE_THREAD_LIBS_INIT} ${ELROB_LIBRARY})
remake_add_headers()
remake_pkg_config_generate(REQUIRES elrob)
