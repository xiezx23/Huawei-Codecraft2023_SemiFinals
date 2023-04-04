#ifndef SOLUTION_HPP
#define SOLUTION_HPP
#include <bits/stdc++.h>
#include "inc_define.hpp"
#include "inc_io.hpp"
#include "inc_tool.hpp"
#include "inc_math_tool.hpp"
#include "inc_mission.hpp"
#include "inc_robot.hpp"
#include "inc_workbench.hpp"
#include "inc_mcmf.hpp"
#include "inc_util.hpp"
#include "inc_dwa.hpp"
#include "inc_shortestpath.hpp"
#include "inc_threadpool.hpp"
#include "inc_pathlock.hpp"
using namespace std;

extern int K;                               // 工作台数
extern int N;                               // 机器人数
extern int frameID;                         // 当前帧
extern int curMoney;                        // 当前金钱
extern char plat[MAP_SIZE][MAP_SIZE + 2];   // 输入地图
extern int totalSellNum[WORKBENCH_SIZE];    // 物品的出售次数
extern const double PI;                     // 圆周率

extern map<int, vector<int>> type2BuyIndex;                     // 根据产品类型寻找收购方下标
extern pair<pair<int,int>,int> profitAndTime[WORKBENCH_SIZE];   // 记录收购价、购入价以及生产用时

extern double para1;
extern double para2;
extern double para4;

double cntPontEnergy(int, coordinate&);
void collitionAvoidance();
void ori_collitionAvoidance();
void ori_solution();
#endif
