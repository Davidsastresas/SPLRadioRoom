/*
 Stopwatch.h

 (C) Copyright 2018 Envirover.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 Created on: Jan 4, 2018
     Author: Pavel Bobov
*/

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <chrono>

using namespace std;
using namespace std::chrono;

/**
 * Stopwatch class is used to measure absolute elapsed time.
 */
class Stopwatch
{
    high_resolution_clock::time_point start_time;

public:

    Stopwatch() : start_time(high_resolution_clock::now())
    {
    }

    /*
     * Get current time.
     */
    high_resolution_clock::time_point time() {
        return high_resolution_clock::now();
    }

    /*
     * Reset stopwatch.
     */
    void reset()
    {
        reset(high_resolution_clock::now());
    }

    /*
     * Set start time.
     */
    void reset(high_resolution_clock::time_point t)
    {
        start_time = t;
    }

    /**
     * Returns number of seconds elapsed from the start of the stopwatch.
     */
    double elapsed_time()
    {
        return duration_cast<milliseconds>(time() - start_time).count() / 1000.0;
    }

};

#endif /* STOPWATCH_H_*/