#ifndef PATHLOCK_HPP
#define PATHLOCK_HPP
#include "inc_codecraft2023.hpp"

extern int lockID[MAP_SIZE][MAP_SIZE];         //该格子对应的锁ID
extern int lockCnt;                            //锁的数量
extern int lockStatus[MAP_SIZE * MAP_SIZE];    //锁的状态
extern char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
extern std::mutex path_mutex;                  //锁的互斥量

// 预处理地图以及对应所变量
void pathlock_init();

// 输出处理后的地图
void printMap();

// 释放锁
void pathlock_release(int rtidx, int x, int y);

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y);

// 判断节点是否可经过
bool pathlock_isReachable(int rtidx, int x, int y);

// 获取锁类型
int pathlock_type(int x,int y);

// 对机器人当前任务路径上的所有点上锁
bool lockPath(int rtidx);

#endif