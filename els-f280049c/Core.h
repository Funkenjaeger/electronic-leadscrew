// Clough42 Electronic Leadscrew
// https://github.com/clough42/electronic-leadscrew
//
// MIT License
//
// Copyright (c) 2019 James Clough
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef __CORE_H
#define __CORE_H

#include "ControlPanel.h"
#include "Tables.h"
#include "cla_shared.h"

class Core
{
private:

public:
    Core();

    void setFeed(const FEED_THREAD *feed);
    void setReverse(bool reverse);
    Uint16 getRPM(void);
    bool isAlarm();

    bool isPowerOn();
    void setPowerOn(bool);
};

inline void Core :: setFeed(const FEED_THREAD *newFeed)
{
#ifdef USE_FLOATING_POINT
    feed = (float)newFeed->numerator / newFeed->denominator;
#else
    feed = newFeed;
#endif // USE_FLOATING_POINT
}

inline Uint16 Core :: getRPM(void)
{
    return rpm_out;
}

inline bool Core :: isAlarm()
{
    return alarm;
}

inline bool Core :: isPowerOn()
{
    return powerOn;
}

#endif // __CORE_H
