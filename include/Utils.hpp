#pragma once
#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <chrono>
#include <iostream>

#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

#define PRINT_CLOCK(msg, t1, t0) \
        std::cerr << msg << TIME_DIFF(t1, t0) << endl;

#endif
