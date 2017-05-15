#pragma once

#include <stdint.h>
#ifndef M_PI
#define M_PI 3.14159
#endif

// turbo-speed trig approximation
float turboatan2(float y, float x);
float turboasin(float x);
float turbosin(float x);
float turbocos(float x);

double turbopow(double a, double b);
int32_t sign(int32_t y);
float sign(float y);
