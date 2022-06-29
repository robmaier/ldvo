##
# This file is part of LDVO.
#
# Copyright 2019 Robert Maier, Technical University of Munich.
# For more information see <https://github.com/robmaier/ldvo>.
# If you use this code, please cite the respective publications as
# listed on the above website.
#
# LDVO is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# LDVO is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with LDVO. If not, see <http://www.gnu.org/licenses/>.
##

#==============================================================================
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12 FATAL_ERROR)
PROJECT(LDVO C CXX)

#==============================================================================
# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE RelWithDebInfo FORCE)
# Custom options
OPTION(BUILD_EXAMPLES "Build example applications" ON)
OPTION(BUILD_DOCUMENTATION "Build Doxygen documentation" ON)
#==============================================================================

#==============================================================================
# Dependencies
#==============================================================================

# include header-only third-party libraries (Eigen, CLI11)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/include/)

# OpenCV
FIND_PACKAGE(OpenCV 4.0.0 COMPONENTS core highgui imgproc REQUIRED)

#==============================================================================
# Header-only library and example applications
#==============================================================================

# ------------------------------------------------------------------------
# C++ flags
IF(NOT (WIN32 AND MSVC))
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
    # Add -O0 to remove optimizations when using gcc
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF()

# enforce C++11 standard
if(CMAKE_VERSION VERSION_LESS 3.1)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    IF(NOT DEFINED CMAKE_CXX_STANDARD OR CMAKE_CXX_STANDARD STREQUAL "98")
        SET(CMAKE_CXX_STANDARD 11)
    ENDIF()
    SET(CMAKE_CXX_STANDARD_REQUIRED ON)
ENDIF()

# ------------------------------------------------------------------------
# interface library
ADD_LIBRARY(ldvo INTERFACE)
TARGET_INCLUDE_DIRECTORIES(ldvo INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------
# install header-only library
INSTALL(TARGETS ldvo EXPORT ldvo)
INSTALL(DIRECTORY include/ldvo DESTINATION include)
INSTALL(EXPORT ldvo FILE ldvo-config.cmake
        DESTINATION share/cmake/ldvo)

# ------------------------------------------------------------------------
# build example applications
IF(BUILD_EXAMPLES)
    # include header-only library
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/)

    # glob header files
    FILE(GLOB ldvo_incs "include/ldvo/*.hpp")
    FILE(GLOB ldvo_detail_inl "include/ldvo/detail/*.inl")
    FILE(GLOB ldvo_math_incs "include/ldvo/math/*.hpp")
    FILE(GLOB ldvo_math_inl "include/ldvo/math/*.inl")
    SET(ldvo_headers ${ldvo_incs} ${ldvo_detail_inl} ${ldvo_math_incs} ${ldvo_math_inl})
    SOURCE_GROUP("ldvo" FILES ${ldvo_headers})

    # add executable (add ldvo includes directly for better editing in editor)
    ADD_EXECUTABLE(ldvo_tracker examples/app_tracker.cpp ${ldvo_headers})
    TARGET_LINK_LIBRARIES(ldvo_tracker ${OpenCV_LIBS})
    TARGET_COMPILE_OPTIONS(ldvo_tracker PRIVATE -std=c++11)
ENDIF()

# ------------------------------------------------------------------------
# build Doxygen documentation
IF(BUILD_DOCUMENTATION)
    ADD_SUBDIRECTORY(doc)
ENDIF()
