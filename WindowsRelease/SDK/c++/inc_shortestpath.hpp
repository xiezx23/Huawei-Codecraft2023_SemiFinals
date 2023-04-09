#ifndef shortestPATH_HPP
#define shortestPATH_HPP
#include "inc_codecraft2023.hpp"

struct coordinate2_hash {
    size_t operator()(const coordinate2& c) const;
};

// 所有工作台的离散坐标
extern std::unordered_map<coordinate2, int, coordinate2_hash> workbenchCoordinate;

// 从机器人i出发的单源最短路径
// rtPathLength存储了从源点到该坐标的距离 
extern double rtPathLength[ROBOT_SIZE][WORKBENCH_SIZE]; 

// 从工作台i出发的单源最短路径
// wbPathLength存储了从源点到该坐标的距离 
extern double wbPathLength[WORKBENCH_SIZE][WORKBENCH_SIZE]; 

// 从机器人i到工作台k的最短路上所需转动角度的总和
extern double rtAngleSum[ROBOT_SIZE][WORKBENCH_SIZE];
// 从工作台i到工作台k的最短路上所需转动角度的总和
extern double wbAngleSum[WORKBENCH_SIZE][WORKBENCH_SIZE];

// 上一次调用dijkstra时机器人的离散坐标
extern coordinate2 robotCoordinate[ROBOT_SIZE];

// 机器人到各离散点的距离
extern double rtPointDis[ROBOT_SIZE][MAP_SIZE][MAP_SIZE];
// 工作台到各离散点的距离
extern double wbPointDis[WORKBENCH_SIZE][MAP_SIZE][MAP_SIZE];

// 初始化位置权重
extern void initWeight();

// 预处理，对点的可达性做判断
void initAccessibility();

// 预处理，计算从机器人及工作台到所有工作台的最短路
void initShortestPath(const coordinate2* oricoordinate);

// 计算从idx号机器人或idx号工作台到所有工作台的最短路（用于寻找到生产工作台的最短路，因此不携带物品）
// 当flag为true时，表示源点为机器人；当flag为false时，表示源点为工作台
void dijkstra(int idx, coordinate2 src, bool flag);

// 机器人rtidx调用dijkstra后，将从当前位置到生产工作台，及从生产工作台到消费工作台的最短路进行压缩，并加入任务队列
bool compress(int rtIdx, coordinate2 src, int startIdx, coordinate2 dest1, int endIdx, coordinate2 dest2);
// 比较方向
inline void cmpdir(coordinate2& dir, const coordinate2& c1, const coordinate2& c2); 

#endif
