# Copyright (C) 2010 Aldebaran Robotics

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(soundprocesslocal)
find_package(qibuild)

option(SOUNDPROCESSING_IS_REMOTE
  "module is compiled as a remote module (ON or OFF)"
  OFF)

set(_srcs
    main.cpp
    alsoundprocessing.h
    alsoundprocessing.cpp
)

if(SOUNDPROCESSING_IS_REMOTE)
  add_definitions(" -DSOUNDPROCESSING_IS_REMOTE ")
  qi_create_bin(soundprocesslocal ${_srcs})
else()
  qi_create_lib(soundprocesslocal SHARED ${_srcs} SUBFOLDER naoqi)
endif()


qi_use_lib(soundprocesslocal ALCOMMON ALAUDIO BOOST)
