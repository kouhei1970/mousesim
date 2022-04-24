#ifndef _VECTOR_H_INCLUDED_
#define _VECTOR_H_INCLUDED_

typedef struct
{
  double x;
  double y;
} point2d_t;

point2d_t rotate2d(point2d_t pt, double angle);
point2d_t move2d(point2d_t pt, point2d_t move);
double dot_product2d(point2d_t vec1, point2d_t vec2);
point2d_t vec_add2d(point2d_t vec1, point2d_t vec2);
point2d_t vec_sub2d(point2d_t vec1, point2d_t vec2);

#endif
