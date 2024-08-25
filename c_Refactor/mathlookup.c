/*
 * mathlookup.c
 *
 *  Created on: Aug. 14, 2024
 *      Author: Hassa
 */
#include "mathlookup.h"


double index_X, sin1, sin2, result2;


void init_arctan_lut(void) {
    int i;

    for (i = 0; i < 256; i++) {
        double x = (double)i / (256 - 1);
        arctan_lut[i] = atan(x);
    }
}


double arctan_fast(double x) {
    int index = (int)(x * (TABLE_SIZE - 1));
    if (index < 0) index = 0;
    if (index >= TABLE_SIZE) index = TABLE_SIZE - 1;
    return arctan_lut[index];
}


void init_trig_lookup(void) {
    int i;

    for (i = 0; i < TABLE_SIZE; i++) {
        double angle = (double)i * (2.0 * M_PI / (double)TABLE_SIZE); // Convert index to radians
        sin_lut[i] = sin(angle);
    }
}

double sin_fast(double angle) {
    // Normalize angle to range [0, 2π)
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }

    // Calculate the index in the LUT
    double index_X = (angle / (2 * M_PI)) * (TABLE_SIZE - 1);
    int index1 = (int)index_X;
    int index2 = index1 + 1;

    // Boundary check to ensure index2 does not go out of bounds
    if (index2 >= TABLE_SIZE) {
        index2 = 0;
    }

    // Linear interpolation between sin_lut[index1] and sin_lut[index2]
    double sin1 = sin_lut[index1];
    double sin2 = sin_lut[index2];
    double result = sin1 + (index_X - index1) * (sin2 - sin1);

    return result;
}

double cos_fast(double angle) {
    return sin_fast(angle + M_PI / 2);
}
