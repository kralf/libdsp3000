cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libdsp3000
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Driver library for the KVH DSP-3000 fiber optic gyro."
  AUTHOR "Pierre Lamon"
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/libdsp3000"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX dsp3000-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libelrob
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libdsp3000 libelrob-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBDSP3000_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBDSP3000_GIT_REVISION
)
