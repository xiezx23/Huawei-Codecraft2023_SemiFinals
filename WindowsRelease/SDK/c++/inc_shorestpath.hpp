#ifndef SHORESTPATH_HPP
#define SHORESTPATH_HPP
#include "inc_codecraft2023.hpp"

struct coordinate2_hash {
    size_t operator()(const coordinate2& c) const;
};

// 记录地图中的障碍物
extern bool obstacle[MAP_SIZE][MAP_SIZE];
// 所有工作台的网格化坐标
extern std::unordered_map<coordinate2, int, coordinate2_hash> workbenchLoc;
// 机器人i到工作台k的最短路径存储在数组 shorestPath[i*WORKBENCH_SIZE+k]中
// 该路径的长度存储于pathLength[i*WORKBENCH_SIZE+k]中
// 该路径的节点数存储于pathSize[i*WORKBENCH_SIZE+k]中
// 路径以逆序存储！！！（从目的地到源）
extern coordinate2 shorestPath[ROBOT_SIZE*WORKBENCH_SIZE][MAP_SIZE*MAP_SIZE];
extern double pathLength[ROBOT_SIZE*WORKBENCH_SIZE];
extern int pathSize[ROBOT_SIZE*WORKBENCH_SIZE];

// 计算从rtidx号机器人到所有工作台的最短路
void dijkstra(int rtidx, const coordinate2& src);
// 计算从rtidx号机器人到指定工作台的最短路
void dijkstra(int rtidx, const coordinate2& src, int wbidx, coordinate2 dest);
// 根据最短路径前驱表更新最短路
void updatePath(int rtidx, const coordinate2& src, int wbidx, coordinate2& dest, const std::vector<std::vector<coordinate2>>& precessor, double dis);
// 预处理时，计算从机器人到所有工作台的最短路
void initShorestPath(const std::vector<coordinate2>& oricoordinate);

// 机器人rtidx调用dijkstra后，对最短路进行压缩
void compress(int rtidx, int wbidx, bool buy, bool sell);
// 比较方向
inline void cmpdir(coordinate2& dir, const coordinate2& c1, const coordinate2& c2); 

#endif