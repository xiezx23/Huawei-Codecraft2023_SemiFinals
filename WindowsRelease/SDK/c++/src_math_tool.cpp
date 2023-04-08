#include "inc_codecraft2023.hpp"

const double PI = acos(-1);

// 向量模
double modulusOfvector(vec& a) {
    return sqrt(a.x*a.x + a.y*a.y);
}

// 向量叉积
double crossProduct(vec& a, vec& b) {
    return a.x * b.y - a.y * b.x;
}

// 向量点乘
double dotProduct(vec& a, vec& b) {
    return a.x * b.x + a.y * b.y;
}

// 两点距离
double dis(coordinate& c1, coordinate& c2) {
    double ans = (c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y);
    return sqrt(ans);
}

// 计算两个向量的夹角
double cntAngle(vec& a, vec& b){
    double angleDiff = acos(max(-1.0,min(1.0, dotProduct(a, b) / (modulusOfvector(a)*modulusOfvector(b)))));
    if (isnan(angleDiff)) {
        double dp = dotProduct(a, b);
        double mxm = modulusOfvector(a)*modulusOfvector(b);
        fprintf(stderr, "dotProduct:%lf\t m(a)*m(b):%lf\t cos:%lf\n",dp, mxm, dp/mxm);
        angleDiff = 0;
    }
    return (angleDiff);
}

// 计算从dir1到dir2需要转动的角度
extern double cntAngle(const coordinate2& dir1, const coordinate2& dir2) {
    return acos((dir1.x * dir2.x + dir1.y * dir2.y) / ((sqrt(dir1.x * dir1.x + dir1.y * dir1.y)) * (sqrt(dir2.x * dir2.x + dir2.y * dir2.y))));
}