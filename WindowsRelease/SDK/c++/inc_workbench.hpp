#ifndef WORKBENCH_HPP
#define WORKBENCH_HPP
#include "inc_codecraft2023.hpp"

struct workbench {
    int type;       // 工作台类型 
    coordinate location;
    int rtime;      // 剩余生产时间 Remaining production time
    int rstatus;    // 原材料格状态 Raw—material status
    int pstatus;    // 产品格状态   Product status
    bool reachable; // 可达标志，记录是否有机器人选取该工作台作为购买目标

    // 判断原材料proType型号格是否为空，空返回false
    bool checkHaveProType(int proType) {
        int s = 1;
        while(proType--) s <<= 1;
        return s & rstatus;
    }
    void setProType(int proType) {
        int s = 1;
        while(proType--) s <<= 1;
        rstatus = s | rstatus;
    }
    // 计算当前已拥有原材料数量
    int rawMaterNum() {
        int r = rstatus, cnt = 0;
        while (r) {
            if (r & 1) cnt++;
            r >>= 1;
        }
        return cnt;
    }
};

extern workbench wb[WORKBENCH_SIZE];  // 工作台
#endif