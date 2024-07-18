// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

#include "hw.h"

#include "TouchScreen.h"

// increase or decrease the touchscreen oversampling. This is a little different
// than you make think: 1 is no oversampling, whatever data we get is
// immediately returned 2 is double-sampling and we only return valid data if
// both points are the same 3+ uses insert sort to get the median value. We
// found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 2

TSPoint::TSPoint(void) { x = y = z = 0; }
/**
 * @brief Construct a new TSPoint::TSPoint object
 *
 * @param x0 The point's X value
 * @param y0 The point's Y value
 * @param z0 The point's Z value
 */
TSPoint::TSPoint(int16_t x0, int16_t y0, int16_t z0) {
  x = x0;
  y = y0;
  z = z0;
}
/**
 * @brief Check if the current point is equivalent to another point
 *
 * @param p1 The other point being checked for equivalence
 * @return `true` : the two points are equivalent
 * `false`: the two points are **not** equivalent
 */
bool TSPoint::operator==(TSPoint p1) {
  return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}
/**
 * @brief Check if the current point is **not** equivalent to another point
 *
 * @param p1 The other point being checked for equivalence

 * @return `true` :the two points are **not** equivalent
 * `false`: the two points are equivalent
 */
bool TSPoint::operator!=(TSPoint p1) {
  return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}
