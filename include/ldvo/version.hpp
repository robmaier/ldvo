/**
* This file is part of LDVO.
*
* Copyright 2019 Robert Maier, Technical University of Munich.
* For more information see <https://github.com/robmaier/ldvo>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* LDVO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LDVO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LDVO. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>

#define LDVO_VERSION_MAJOR 1
#define LDVO_VERSION_MINOR 0
#define LDVO_VERSION_REVISION 0

namespace ldvo
{

    /**
     * @brief   Returns the LDVO version as a string.
     * @author  Robert Maier (robert.maier@tum.de)
     */
    inline static const std::string version()
    {
        return std::to_string(LDVO_VERSION_MAJOR) + "." +
               std::to_string(LDVO_VERSION_MINOR) + "." +
               std::to_string(LDVO_VERSION_REVISION);
    }

} // namespace ldvo
