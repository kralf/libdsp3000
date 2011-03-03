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
  HOME "http://aslforge.ethz.ch/projects/libdsp3000"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX dsp3000-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  libdsp3000 libelrob
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  libdsp3000 libelrob-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
