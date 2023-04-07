#ifndef MATH_TOOL_HPP
#define MATH_TOOL_HPP
#include "inc_codecraft2023.hpp"

extern double dis(coordinate& c1, coordinate& c2);
extern double crossProduct(vec& a, vec& b);
extern double dotProduct(vec& a, vec& b);
extern double modulusOfvector(vec& a);
extern double cntAngle(vec& a, vec& b);
extern double cntAngle(const coordinate2& dir1, const coordinate2& dir2);

#endif