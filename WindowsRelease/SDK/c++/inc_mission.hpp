#ifndef MISSION_HPP
#define MISSION_HPP
#include "inc_codecraft2023.hpp"

/*** 
定义任务 
m(A, x, B) 表示把物品x从A工作台购入并出售给B工作台
任务价值函数 v(m) 表示完成任务m可以获得的潜在收益
    1：v 跟x在AB的差价正相关
    2：跟B的剩余原材料格、AB路径负相关
****/

struct mission {
    int startIndex; // 起点工作台下标
    int endIndex;   // 终点工作台下标
    int proType;    // 产品型号
    double v = 0;   // 价值函数
    double estFrame = 0; // 估计任务消耗帧数
    
    mission(){};
    mission(int s, int e, int p) {set(s,e,p);}
    void set(int s, int e, int p);
    void countValue(int rtidx, int proType, vec& lsp);
};

#endif