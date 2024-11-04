# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Yao/esp/v5.1.4/esp-idf/components/bootloader/subproject"
  "D:/Firmware/DragonBallRadar1/build/bootloader"
  "D:/Firmware/DragonBallRadar1/build/bootloader-prefix"
  "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/tmp"
  "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/src"
  "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Firmware/DragonBallRadar1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
