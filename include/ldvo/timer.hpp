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

#include <ctime>


namespace ldvo
{

    /**
     * @brief   Timer class for measuring time.
     * @author  Robert Maier (robert.maier@tum.de)
     */
    class Timer
    {
    public:

        /// Constructs a timer (initially stopped).
        Timer() :
            start_time_(0),
            elapsed_(0.0),
            stopped_(true)
        {
        }


        /// Destructor.
        ~Timer()
        {
        }


        /**
         * @brief   Start the time measurement.
         */
        void start()
        {
            stopped_ = false;
            start_time_ = std::clock();
        }


        /**
         * @brief   Stop the time measurement.
         */
        void stop()
        {
            stopped_ = true;
            elapsed_ = elapsedSinceStart();
        }


        /**
         * @brief   Get the measured time.
         * @return  Elapsed time.
         *          If in stopped state, the time between start and stop
         *          is returned, otherwise the elapsed time since the
         *          last start is returned.
         */
        double elapsed() const
        {
            if (stopped_)
                return elapsed_;
            else
                return elapsedSinceStart();
        }

    private:

        /// Computes the elapsed time since the last timer start.
        double elapsedSinceStart() const
        {
            clock_t end = std::clock();
            return double(end - start_time_) / CLOCKS_PER_SEC;
        }

        clock_t start_time_;
        double elapsed_;
        bool stopped_;
    };

} // namespace ldvo
