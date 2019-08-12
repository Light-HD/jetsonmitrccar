#ifndef MATH_UTILS_LOW_LEVEL_CONTROLLER_H
#define MATH_UTILS_LOW_LEVEL_CONTROLLER_H


inline double clamp_value(double a, double max, double min){
    if(a < min){
        a = min;
    }

    if(a > max){
        a = max;
    }
    
    return a;
}
#endif
