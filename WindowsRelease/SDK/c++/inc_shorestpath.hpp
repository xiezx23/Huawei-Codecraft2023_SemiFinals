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
// 四维向量，shortestPath[x][y][k][i]表示坐标(x,y)到工作台k的第i个节点的坐标
extern std::vector<std::vector<std::vector<std::vector<coordinate2>>>>  shorestPath;

// 计算从机器人初始位置到达所有工作台的最短路
void initShorestPath(const std::vector<coordinate2>& oricoordinate);
// 根据最短路径前驱表更新最短路
void updatePath(coordinate2 src, coordinate2 dest, const std::vector<std::vector<coordinate2>>& precessor);
// 计算从src坐标到所有工作台的最短路
void dijkstra(coordinate2 src);

#endif