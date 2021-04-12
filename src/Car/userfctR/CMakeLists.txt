#
#   Universite catholique de Louvain
#   CEREM : Centre for research in mechatronics
#   http://www.robotran.be  
#   Contact : info@robotran.be
#
#
# CMake for compiling the user files of a robotran project in C
# 

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       LIBRARY MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

if (UNIX)
    cmake_minimum_required(VERSION 2.8.7)
else()
    cmake_minimum_required(VERSION 3.4)
endif ()

if(NOT DEFINED USERFCT_LIB_NAME)
    set (USERFCT_LIB_NAME "Project_userfct")
endif ( )

project(${USERFCT_LIB_NAME})

# flags
if (UNIX)
    set(CMAKE_C_FLAGS "-fPIC")
    set(CMAKE_EXE_LINKER_FLAGS "-fPIC")
endif (UNIX)

# Windows M_PI definitions
if (WIN32)
    add_definitions(-D _USE_MATH_DEFINES)
endif (WIN32)

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                        SEPARATE COMPILATION
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

if ( FLAG_SEPARATE_USER_FCT )

    # additional CMakeLists.txt
    set(CMAKE_AUX ${ROBOTRAN_SOURCE_DIR}/cmake_aux)
    set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)

    add_subdirectory ( ${CMAKE_AUX}/flags/   ${CMAKE_AUX_BIN}/flags/   )
    add_subdirectory ( ${CMAKE_AUX}/listing/ ${CMAKE_AUX_BIN}/listing/ )
    add_subdirectory ( ${CMAKE_AUX}/libraries/ ${CMAKE_AUX_BIN}/libraries/ )

    # real-time option
    option (FLAG_REAL_TIME "Real time" ON)

    if(FLAG_REAL_TIME)
        add_definitions( -DREAL_TIME )

        # configure a header file to pass some of the CMake settings to the source code
        configure_file (
            "${ROBOTRAN_SOURCE_DIR}/conf/cmake_config.h.in"
            "${PROJECT_BINARY_DIR}/conf/cmake_config.h"
        )
        include_directories (${PROJECT_BINARY_DIR}/conf)

        # plot-visu options
        option (FLAG_PLOT "Real time" ON)
        option (FLAG_VISU "Real time" ON)
        option (FLAG_JAVA "Real time" ON)
        option (FLAG_OPEN_GL "Real time" ON)

        if(FLAG_PLOT)
            add_definitions( -DSDL )
        endif( )

        if(FLAG_VISU)
            add_definitions( -DVISU_3D )
        endif( )

        if(FLAG_JAVA)
            add_definitions( -DJAVA )
        endif( )

        if(FLAG_OPEN_GL)
            add_definitions( -DOPEN_GL )
        endif( )

    endif ( )
endif ( )

# release of debug
release_debug()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         SET LIBRARY
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# list source files to compile
init_src()
increment_src( ${PROJECT_SOURCE_DIR} )
increment_void_user( ${PROJECT_SOURCE_DIR} )

# list include directories (to find headers)
init_include()
increment_include( ${ROBOTRAN_SOURCE_DIR}/mbs_common )

# include these directories
include_directories ( ${INCLUDE_DIR} )
include_directories ( ${CMAKE_CURRENT_BINARY_DIR} )

if( FLAG_SHARED_LIB OR FLAG_SEPARATE_USER_FCT )
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_library(${USERFCT_LIB_NAME} SHARED ${SOURCE_FILES})

    target_link_libraries (${USERFCT_LIB_NAME} ${LIB_MBSYSC_REALTIME})
    target_link_libraries (${USERFCT_LIB_NAME} ${LIB_MBSYSC_MODULES})
    target_link_libraries (${USERFCT_LIB_NAME} ${LIB_MBSYSC_UTILITIES})
    
    # MacOS: Force extension to be .so rather than .dylib
    if(APPLE)
        set_target_properties(${USERFCT_LIB_NAME} PROPERTIES SUFFIX ".so")    
    endif()

    # Windows, copy dll next to executable
    if((NOT UNIX) AND (NOT FLAG_SEPARATE_USER_FCT))
        add_custom_command(TARGET ${USERFCT_LIB_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/${USERFCT_LIB_NAME}.dll
                ${CMAKE_CURRENT_BINARY_DIR}/../$<CONFIGURATION>/${USERFCT_LIB_NAME}.dll
        )
    endif()
    
else()
    add_library(${USERFCT_LIB_NAME} STATIC ${SOURCE_FILES})
    target_link_libraries (${USERFCT_LIB_NAME} ${LIB_MBSYSC_REALTIME} ${LIB_MBSYSC_UTILITIES})
endif()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         LINK LIBRARIES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# MBSysC libraries
if ( FLAG_SEPARATE_BUILD ) # find MBSysC dynamic libraries
    find_package( LibRobotranC 1.12.0 REQUIRED )
    add_definitions(${LIB_MBSYSC_DEFINITIONS})
endif()

if (FLAG_PLOT)
    sdl_header_lib(userfct)
endif ( )

