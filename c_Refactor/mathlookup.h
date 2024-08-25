/*
 * mathlookup.h
 *
 *  Created on: Aug. 14, 2024
 *      Author: Hassa
 */
#include <math.h>
#include <stdint.h>

#pragma once


#define TABLE_SIZE 2042
double arctan_lut[256];
double sin_lut[TABLE_SIZE];

void init_arctan_lut();
void init_trig_lookup();

double arctan_fast(double x);
double sin_fast(double angle);
double cos_fast(double angle);

