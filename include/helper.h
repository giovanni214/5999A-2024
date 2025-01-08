#ifndef HELPER_H
#define HELPER_H

#include "pros/optical.hpp"

// Converts a hue to RGB value
float hueToRGB(float p, float q, float t);

// Converts HSL to RGB
void hslToRGB(float h, float s, float l, float& r, float& g, float& b);

//Gives ring color (Red or Blue)
int getRingColor(pros::Optical& optical_sensor);

float centiToDegrees(int centidegrees);

#endif