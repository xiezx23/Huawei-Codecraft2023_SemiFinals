#ifndef TOOL_HPP
#define TOOL_HPP
#include "inc_codecraft2023.hpp"

#define vec coordinate

struct coordinate {
    double x, y;
    coordinate() {};
    coordinate(double xx, double yy): x(xx), y(yy) {}
    void set(double xx, double yy) {x = xx; y = yy;}
};

// 将连续坐标离散化，将连续坐标映射到 100 * 100的离散点中
struct coordinate2 {
    int x, y;
    coordinate2(): x(0), y(0) {}
    coordinate2(int xx, int yy): x(xx), y(yy) {}
    coordinate2(coordinate c): x((c.x + 0.24999)/0.5), y((c.y + 0.24999)/0.5) {}    
    inline bool operator==(const coordinate2& c) const {
        return (c.x == x && c.y == y);
    }
    inline bool operator!=(const coordinate2& c) const {
        return (c.x != x || c.y != y);
    }
    inline void set(int xx, int yy) {x = xx; y = yy;}
    inline operator coordinate() const { return coordinate(x*0.5+0.25, y*0.5+0.25); }
};

struct node {
    double distance;
    coordinate2 coor;
    node(double d, coordinate2 c): distance(d), coor(c) {}
    inline bool operator>(const node& n) const {
        return distance > n.distance;
    }
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