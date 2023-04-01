#include "inc_codecraft2023.hpp"

void mission::set(int s, int e, int p) {
    startIndex = s;
    endIndex = e;
    proType = p;
}

void mission::countValue(int rtidx, int proType, vec& lsp) {
    coordinate& rtCo = rt[rtidx].location;
    // 计算价值函数 参数依次为机器人坐标、预计携带产品类型、机器人速度向量
    coordinate s = wb[startIndex].location;
    coordinate e = wb[endIndex].location;
    // double dd = dis(rtCo, s) + dis(s, e);   // 机器人到起点再到终点的距离    
    double dd = pathLength[rtidx*WORKBENCH_SIZE+startIndex] + pathLength[rtidx*WORKBENCH_SIZE+endIndex];
    if (!pathSize[rtidx*WORKBENCH_SIZE+startIndex]) dd = INF;
    if (!pathSize[rtidx*WORKBENCH_SIZE+endIndex]) dd = INF;
    vec r2s(s.x - rtCo.x, s.y - rtCo.y);    // 机器人到起点向量
    vec s2e(e.x - s.x, e.y - s.y);          // 起点到终点的向量
    double rr = cntAngle(lsp, r2s) + cntAngle(r2s, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI;
    estFrame = tt * 50 + 50;
    double vv = profitAndTime[proType].first.first - profitAndTime[proType].first.second;
    double nextVv = profitAndTime[wb[endIndex].type].first.first - profitAndTime[wb[endIndex].type].first.second;
    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
    }
    else if (wb[endIndex].type == 7) {
        vv += para4*nextVv/(3-wb[endIndex].rawMaterNum());
    }
    else if (wb[endIndex].type > 3) {
        vv += para4*nextVv/(2-wb[endIndex].rawMaterNum());
    }
    v = para1 / tt + para2 * vv;
} 
