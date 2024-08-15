# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/elias/esp/esp-idf/components/bootloader/subproject"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/tmp"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/src/bootloader-stamp"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/src"
  "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/elias/workspace/Kunder/Devent/timergate/Software/timergate-ap/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
