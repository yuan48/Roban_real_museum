cmake_minimum_required(VERSION 2.8)
project(aiui_sample)
set(CMAKE_CXX_STANDARD 11)

# 必须指定编译平台
if (NOT DEFINED PLATFORM)
    MESSAGE(FATAL_ERROR "PLATFORM can be x64 or x86")
endif (NOT DEFINED PLATFORM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type: Release" FORCE)
endif()

SET(COMMON_FLAG "-w -std=c++11")

IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    MESSAGE(STATUS "current platform: Linux  64")
    IF (PLATFORM STREQUAL "x86")
        MESSAGE(STATUS "build 32 bit lib in 64 os system")
        SET(COMMON_FLAG "${COMMON_FLAG} -m32")
    ENDIF (PLATFORM STREQUAL "x86")
ELSE ()
    IF (PLATFORM STREQUAL "x64")
        MESSAGE(FATAL_ERROR "can not build 64bit on 32 os system")
    ENDIF ()
ENDIF (CMAKE_SIZEOF_VOID_P EQUAL 8)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAG}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAG}")

include_directories(${CMAKE_CURRENT_LIST_DIR}/../include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/jsoncpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAIUI_LINUX")

link_directories(${CMAKE_CURRENT_LIST_DIR}/../libs/${PLATFORM})

link_libraries(aiui)

add_executable(${PROJECT_NAME}
                    src/demo.cpp

                    src/jsoncpp/json_reader.cpp
                    src/jsoncpp/json_value.cpp
                    src/jsoncpp/json_writer.cpp
                    )

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/../bin_${PLATFORM})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

# 去除cmake自带的rpath，改写为应用的所在目录
SET(CMAKE_SKIP_RPATH TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath,\$ORIGIN:\$ORIGIN/libs")

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/AIUI"
        "${EXECUTABLE_OUTPUT_PATH}/AIUI")
		
add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/../libs/${PLATFORM}"
        "${EXECUTABLE_OUTPUT_PATH}")