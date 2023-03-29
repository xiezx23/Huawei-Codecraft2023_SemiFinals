#ifndef ROBOT_HPP
#define ROBOT_HPP
#include "inc_codecraft2023.hpp"

struct robot {
    int rtIdx;
    int wb_id;              // 所处工作台ID
    int pd_id;              // 携带产品类型
    double tvc;             // 时间价值系数 Time value coefficient
    double cvc;             // 碰撞价值系数 Collision value coefficient
    double pcvc;            // 前一帧的碰撞价值系数 Collision value coefficient
    double asp;             // Angular speed
    vec lsp;                // Linear speed
    double toward;          // 朝向，弧度制
    coordinate location;
    command cmd;            // 当前帧要发布的控制指令
    void setSpeed(coordinate dest); // 负责从当前位置移动到目的地的线速度和角速度指令

    std::queue<task> taskQueue; // 任务队列
    mission curMission;

    // 碰撞避免持续时间
    int holdTime = 0;
    bool leftOrRight = 0; // left:0, right:1
    vec avoidance;
    // 势能场临时中间点碰撞避免所需变量
    coordinate temDest;
    bool haveTemDest = 0;

    // 当前任务
    task curTask;
    //携带的产品来自的节点
    int nodeId = -1;

    void checkDest();
    void checkTask();
    void checkSpeed();
    void findMission(std::vector<mission>&, coordinate&, vec&);
    void setTemporaryDest(coordinate&); // 设置临时目的地

    // 用于统计
    void collisionCount();
    void buysellCount();
};

extern robot rt[ROBOT_SIZE];          // 机器人
#endif