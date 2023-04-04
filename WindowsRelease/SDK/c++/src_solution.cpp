#include "inc_codecraft2023.hpp"

/******************************
author:     xiezx
date:       2023-3-22
describe:   
    1、价值函数加入旋转时间考量
    2、降低直接把产品送到8、9号工作台的权重，优先用于合成
    3、估算任务需要帧长度，尽量保证在9000帧前完成已分配的任务
    4、加入收购工作台剩余原材料空格数量对价值的影响，剩余越少空格越重视
******************************/

void robotWork(void* a) {
    int rtIdx = *(int*)a;    
    // if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
    rt[rtIdx].cmd.clean(); // 清除之前指令设置
    rt[rtIdx].checkDest(); // 检查是否到达目的地
    rt[rtIdx].checkTask(); // 任务执行->运动指令
    // vec motion = motionPredict(rtIdx);
    // rt[rtIdx].cmd.forward = motion.x;
    // rt[rtIdx].cmd.rotate = motion.y;
}

void ori_solution() {
    // 根据已分配任务把工作台信息进行同步
    for (int rtIdx = 0; rtIdx < ROBOT_SIZE; ++rtIdx) {
        rt[rtIdx].checkSpeed(); // 保证速度非0
        if (rt[rtIdx].taskQueue.size()) {
            mission& tmp = rt[rtIdx].curMission;
            wb[tmp.endIndex].setProType(tmp.proType);
        }
    }
    // 指令规划
    for (int rtIdx = 0; rtIdx < ROBOT_SIZE; ++rtIdx) {        
        // if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
        rt[rtIdx].cmd.clean(); // 清除之前指令设置
        rt[rtIdx].checkDest(); // 检查是否到达目的地
        rt[rtIdx].checkTask(); // 任务执行->运动指令
    //     vec motion = motionPredict(rtIdx);
    //     rt[rtIdx].cmd.forward = motion.x;
    //     rt[rtIdx].cmd.rotate = motion.y;
    }
    // int rtIdxArr[4];
    // for (int rtIdx = 0; rtIdx < ROBOT_SIZE; ++rtIdx) {
        // rtIdxArr[rtIdx] = rtIdx;
        // tp->addWork(&robotWork, (void*)&rtIdxArr[rtIdx]);
    // }
    // collitionAvoidance();
    // tp->waitFinish();
    // 碰撞避免
    // ori_collitionAvoidance(); 
    return;
}