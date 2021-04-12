#
#   Universite catholique de Louvain
#   CEREM : Centre for research in mechatronics
#   http://www.robotran.be  
#   Contact : info@robotran.be
#
#
# CMake for compiling the symbolic files of a robotran project in C
# 

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       LIBRARY MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

if (UNIX)
    cmake_minimum_required(VERSION 2.8.7)
else()
    cmake_minimum_required(VERSION 3.4)
endif ()

if(NOT DEFINED SYMBOLIC_LIB_NAME)
    set (SYMBOLIC_LIB_NAME "Project_symbolic")
endif ( )

project(${SYMBOLIC_LIB_NAME})

# flags
if (UNIX)
    set(CMAKE_C_FLAGS "-fPIC")
    set(CMAKE_EXE_LINKER_FLAGS "-fPIC")
endif (UNIX)

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                        SEPARATE COMPILATION
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

if ( FLAG_SEPARATE_SYMBOLIC )
    set(CMAKE_AUX ${ROBOTRAN_SOURCE_DIR}/cmake_aux)
    set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)

    add_subdirectory ( ${CMAKE_AUX}/flags/   ${CMAKE_AUX_BIN}/flags/   )
    add_subdirectory ( ${CMAKE_AUX}/listing/ ${CMAKE_AUX_BIN}/listing/ )
endif ( )

# release of debug
release_debug()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         SET LIBRARY
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# list source files to compile
init_src()
increment_src( ${PROJECT_SOURCE_DIR} )
increment_void_symbolic( ${PROJECT_SOURCE_DIR} )

# list include directories (to find headers)
init_include()
increment_include( ${ROBOTRAN_SOURCE_DIR}/mbs_common )

# include these directories
include_directories ( ${INCLUDE_DIR} )
include_directories ( ${CMAKE_CURRENT_BINARY_DIR} )

if( FLAG_SHARED_LIB OR FLAG_SEPARATE_SYMBOLIC )

    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_library(${SYMBOLIC_LIB_NAME} SHARED ${SOURCE_FILES})
    
    # MacOS: Force extension to be .so rather than .dylib
    if(APPLE)
        set_target_properties(${SYMBOLIC_LIB_NAME} PROPERTIES SUFFIX .so)    
    endif()    

    # Windows, copy dll next to executable
    if((NOT UNIX) AND (NOT FLAG_SEPARATE_SYMBOLIC))
        add_custom_command(TARGET ${SYMBOLIC_LIB_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                        ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/${SYMBOLIC_LIB_NAME}.dll
                        ${CMAKE_CURRENT_BINARY_DIR}/../$<CONFIGURATION>/${SYMBOLIC_LIB_NAME}.dll
        )
    endif()

else()
    add_library(${SYMBOLIC_LIB_NAME} STATIC ${SOURCE_FILES})

    # some symbolic routines call user functions (ext_forces, accelred, ...)
    target_link_libraries(${SYMBOLIC_LIB_NAME} Project_userfct)
endif()


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         LINK LIBRARIES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# MBSysC libraries
if ( FLAG_SEPARATE_BUILD ) # find MBSysC dynamic libraries
    find_package( LibRobotranC 1.12.0 REQUIRED )
    add_definitions(${LIB_MBSYSC_DEFINITIONS})
    target_link_libraries(${SYMBOLIC_LIB_NAME} ${LIB_MBSYSC_MODULES} ${LIB_MBSYSC_MODULES})
endif()

# Windows M_PI definitions
if (WIN32)
    add_definitions(-D _USE_MATH_DEFINES)
endif (WIN32)
