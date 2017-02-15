#pragma once

#include <stdint.h>
#ifndef M_PI
#define M_PI 3.14159
#endif

// float-based wrappers
float turboatan2(float y, float x);
float turboasin(float x);

// turbo-speed trig approximation
int32_t atan2_lookup(int32_t y, int32_t x);
int32_t atan_lookup(int32_t x);
int32_t asin_lookup(int32_t x);

// Low and Medium Precision Taylor Series Approximations
double turbosinlp(double x);
double turbocoslp(double y);
double turbosinmp(double x);
double turbocosmp(double x);

double turbopow(double a, double b);
int32_t sign(int32_t y);
float sign(float y);
