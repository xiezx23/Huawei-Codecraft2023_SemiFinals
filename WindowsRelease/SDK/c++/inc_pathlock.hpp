#ifndef PATHLOCK_HPP
#define PATHLOCK_HPP
#include "inc_codecraft2023.hpp"


// 维护基于时间窗的锁
struct pathlock_node{
    int s_time, e_time, rtIdx;
    pathlock_node(){}
    pathlock_node(int s_time, int e_time, int rtIdx) : s_time(s_time), e_time(e_time), rtIdx(rtIdx) {}
    bool operator < (const pathlock_node &rhs) const {
        if (s_time != rhs.s_time) return s_time < rhs.s_time;
        if (e_time != rhs.e_time) return e_time < rhs.e_time;
        return rtIdx < rhs.rtIdx;
    }
    void set(int s_time, int e_time, int rtIdx) {
        this->s_time = s_time;
        this->e_time = e_time;
        this->rtIdx = rtIdx;
    }
    bool check(const pathlock_node &rhs) const {
        return (rhs.rtIdx != rtIdx && rhs.e_time >= s_time && e_time >= rhs.s_time);
    }
};

extern std::set<pathlock_node> lockStatus[MAP_SIZE * MAP_SIZE]; //锁的状态
extern int lockSize[MAP_SIZE * MAP_SIZE];           //锁的大小
extern bool lockWindows[ROBOT_SIZE];                //临时数组

extern int lockID[MAP_SIZE][MAP_SIZE];         //该格子对应的锁ID
extern int lockCnt;                            //锁的数量
extern char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
extern std::mutex path_mutex;                  //锁的互斥量

// 预处理地图以及对应所变量
void pathlock_init();

// 输出处理后的地图
void printMap();

// 获取锁类型
int pathlock_type(int x,int y);

// 获取锁类型
int pathlock_type(const coordinate2 &pos);

// 判断目标时间窗能否被加锁
bool pathlock_isLockable(int rtidx, const pathlock_node &rhs, int id);

// 释放锁,type = 0 顺序释放锁，type = 1 倒序释放锁
void pathlock_release(int rtidx, int x, int y, int flag = 0);

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y, int s_time, int e_time);

// 判断节点在指定时间段是否可经过
bool pathlock_isReachable(int rtidx, int x, int y, int s_time, int e_time);

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y, const coordinate2& time);

// 获取锁
bool pathlock_acquire(int rtidx, const coordinate2& pos, const coordinate2& time);

// 判断节点在指定时间段是否可经过
bool pathlock_isReachable(int rtidx, int x, int y, const coordinate2& time);


// 释放锁,type = 0 顺序释放锁，type = 1 倒序释放锁
void pathlock_release(int rtidx, const coordinate2 &pos, int flag = 0);

// dijkstra调用时预估的时间
coordinate2 pathlock_getExpectTime(double dd);

// 上锁时预估的时间,size用于表示剩余加入栈的节点数量
coordinate2 pathlock_getExpectTime(double dd , int size);
#endif 