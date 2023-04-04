#ifndef shortestPATH_HPP
#define shortestPATH_HPP
#include "inc_codecraft2023.hpp"

struct coordinate2_hash {
    size_t operator()(const coordinate2& c) const;
};

// 所有工作台的离散坐标
extern std::unordered_map<coordinate2, int, coordinate2_hash> workbenchLoc;

// 从机器人i出发的单元最短路径
// pathLength存储了从源点到该坐标的距离
extern double pathLength[ROBOT_SIZE][WORKBENCH_SIZE]; 

// 初始化位置权重
extern void initWeight();

// 预处理时，计算从机器人到所有工作台的最短路
void initShortestPath(const coordinate2* oricoordinate);

// 计算从rtidx号机器人到所有工作台的最短路（用于寻找到生产工作台的最短路，因此不携带物品）
void dijkstra(int rtidx, coordinate2 src);
// 计算从rtidx号机器人到指定工作台的最短路（用于寻找到消耗工作台的最短路，携带了物品）
void dijkstra(int rtidx, coordinate2 src, int wbidx, coordinate2 dest);

// 机器人rtidx调用dijkstra后，对最短路进行压缩
bool compress(int rtidx, coordinate2 src, int wbidx, coordinate2 dest, bool buy, bool sell);
// 比较方向
inline void cmpdir(coordinate2& dir, const coordinate2& c1, const coordinate2& c2); 

#endif
