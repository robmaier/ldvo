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

# find Doxygen for building documentation
FIND_PACKAGE(Doxygen)

IF(DOXYGEN_FOUND)
    # Doxygen found

    # set up Doxygen configuration file
    SET(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in)
    SET(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)
    CONFIGURE_FILE(${doxyfile_in} ${doxyfile} @ONLY)

    # add custom target for buildign Doxygen documentation
    ADD_CUSTOM_TARGET(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

    # install generated documentation
    INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
ELSE()
    # Doxygen not found
    MESSAGE(FATAL_ERROR "Doxygen is needed to build the documentation.")
ENDIF()
