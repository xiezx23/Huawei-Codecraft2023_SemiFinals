#ifndef UTIL_HPP
#define UTIL_HPP
#include "inc_codecraft2023.hpp"

// 日志信息

struct logInfo {
private:
    int wallCollisionNum[ROBOT_SIZE];         // 跟墙壁碰撞次数
    int roboCollisionNum[ROBOT_SIZE];         // 跟其他机器人碰撞次数
    int buyNum[ROBOT_SIZE][WORKBENCH_SIZE];   // 物品的购买次数
    int sellNum[ROBOT_SIZE][WORKBENCH_SIZE];  // 物品的出售次数
public:
    logInfo() {
        for (int i = 0; i < ROBOT_SIZE; ++i) {
            wallCollisionNum[i] = 0;
            roboCollisionNum[i] = 0;
            memset(buyNum, 0, sizeof(buyNum[0]));
            memset(sellNum, 0, sizeof(sellNum[0]));
        }
    }
    void addWallCol(int);
    void addRoboCol(int);
    void buyProduct(int, int);
    void sellProduct(int, int);
    void printLog();
};

extern logInfo dataLog;
#endif