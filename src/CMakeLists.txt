##
##   ______                              _
##  / _____)             _              | |
## ( (____  _____ ____ _| |_ _____  ____| |__
##  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
##  _____) ) ____| | | || |_| ____( (___| | | |
## (______/|_____)_|_|_| \__)_____)\____)_| |_|
## (C)2019 Semtech
##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
##
## Authors: Semtech WSP Applications Team
##
project(llcc68_driver VERSION 0.9.0 DESCRIPTION "LLCC68 driver")
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------

list(APPEND ${PROJECT_NAME}_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/llcc68.c
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

# For debug builds set the symbol DEBUG
set(CMAKE_C_FLAGS_DEBUG -DDEBUG)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
