#include "vector.h"
#include "math.h"

//vec1+vec2
point2d_t vec_add2d(point2d_t vec1, point2d_t vec2)
{
  point2d_t vec;
  vec.x = vec1.x + vec2.x;
  vec.y = vec1.y + vec2.y;
  return vec;
}

//vec1-vec2
point2d_t vec_sub2d(point2d_t vec1, point2d_t vec2)
{
  point2d_t vec;
  vec.x = vec1.x + vec2.x;
  vec.y = vec1.y + vec2.y;
  return vec;
}


point2d_t rotate2d(point2d_t pt, double angle)
{
  point2d_t p;
  p.x = cos(angle)*pt.x - sin(angle)*pt.y;
  p.y = sin(angle)*pt.x + cos(angle)*pt.y;
  return p;
}

point2d_t move2d(point2d_t pt, point2d_t move)
{
  point2d_t p;
  p.x = pt.x + move.x;
  p.y = pt.y + move.y;
  return p;
}

double dot_product2d(point2d_t vec1, point2d_t vec2)
{
  return vec1.x * vec2.x + vec1.y * vec2.y;
}
