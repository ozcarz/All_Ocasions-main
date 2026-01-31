#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>

inline int clampPct(int val) {
    return std::max(-100, std::min(100, val));
}

inline double clampPctD(double val) {
    return std::max(-100.0, std::min(100.0, val));
}

inline double absD(double val) {
    return std::fabs(val);
}

inline int absI(int val) {
    return std::abs(val);
}

inline double clampD(double val, double minVal, double maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

inline int clampI(int val, int minVal, int maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

#endif