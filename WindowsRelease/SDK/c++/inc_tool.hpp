#ifndef TOOL_HPP
#define TOOL_HPP
#include "inc_codecraft2023.hpp"

#define vec coordinate

struct coordinate {
    double x, y;
    coordinate() {};
    coordinate(double xx, double yy) {x = xx; y = yy;}
    void set(double xx, double yy) {x = xx; y = yy;}
};

struct command { // 汇总当前帧机器人的控制指令
    double forward = 0;
    double rotate = 0;
    bool buy = false;
    bool sell = false;
    bool destroy = false;
    inline void clean() {
        buy = sell = destroy = false;
    }
};

struct task { // 机器人的当前目标工作
    coordinate destCo;  // 目的坐标
    int destId;         // 目标工作台下标
    bool buy = false;
    bool sell = false;
    task(coordinate c, int d, bool b, bool s) {
        destCo = c;
        destId = d;
        buy = b;
        sell = s;
    }
    task():destId(-1),destCo(0,0){}
};


#endif