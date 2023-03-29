#ifndef TOOL_HPP
#define TOOL_HPP
#include <functional>
#include "inc_codecraft2023.hpp"

#define vec coordinate

struct coordinate {
    double x, y;
    coordinate() {};
    coordinate(double xx, double yy) {x = xx; y = yy;}
    void set(double xx, double yy) {x = xx; y = yy;}
};

// 将物理坐标网格化，将物理实际坐标映射到 100 * 100的网格中
struct coordinate2 {
    int x, y;
    coordinate2() {};
    coordinate2(double xx, double yy): x(xx/0.5), y(yy/0.5) {}
    coordinate2(int xx, int yy): x(xx), y(yy) {}
    bool operator==(const coordinate2& c) const {
        return (c.x == x && c.y == y);
    }
    bool operator!=(const coordinate2& c) const {
        return (c.x != x || c.y != y);
    }
    void set(int xx, int yy) {x = xx; y = yy;}
};

struct node {
    double distance;
    coordinate2 coor;
    node(double d, coordinate2 c): distance(d), coor(c) {}
    bool operator>(const node& n) const {
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