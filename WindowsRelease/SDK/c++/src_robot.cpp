#include "inc_robot.hpp"

// 统计碰撞次数
void robot::collisionCount() {    
    if ((pcvc - cvc >= 0.001) && (cvc > 0.79)) {
        double robotRadius = 0.45;  // 机器人初始半径
        if (pd_id) {                // 携带物品时半径加大
            robotRadius = 0.53;
        }
        robotRadius += 0.1;
        // 判断碰撞原因
        if (fabs(location.x) <= robotRadius || fabs(location.x - 50.0) <= robotRadius) {
            dataLog.addWallCol(rtIdx);
        }
        else if (fabs(location.y) <= robotRadius || fabs(location.y - 50.0) <= robotRadius) {
            dataLog.addWallCol(rtIdx);
        }
        else dataLog.addRoboCol(rtIdx);
    }
}

// 统计购买与出售次数
void robot::buysellCount() {
    if (cmd.buy)  dataLog.buyProduct(rtIdx, wb[wb_id].type);
    if (cmd.sell) dataLog.sellProduct(rtIdx, pd_id);
}

// 设置临时目的地
void robot::setTemporaryDest(coordinate& td) {
    temDest = td;
    haveTemDest = true;
    // 立即前往临时目的地
    setSpeed(temDest);
}

// 检查是否到达目的地
void robot::checkDest() {
    if (haveTemDest) { 
        // 检查是否到达临时目的地附近
        if (dis(temDest, location) < 0.5) {
            // 视为到达
            haveTemDest = false;
        }
    }
    auto releaseLock = [&](const coordinate2 & t) {
        std::unique_lock<std::mutex> lock(path_mutex);
        pathlock_release(rtIdx, t.x, t.y);
    };

    if (!taskQueue.empty()) {
        curTask = taskQueue.front();
        double radium = pd_id ? 0.53 : 0.48;
        if (!curTask.buy && !curTask.sell) {
            if (dis(curTask.destCo, location) < radium + 0.1) {
                releaseLock(curTask.destCo);
                taskQueue.pop();
            }            
        }
        else if (wb_id == curTask.destId) {
            // 到达当前工作目的地，交付工作
            if (curTask.buy && wb[wb_id].pstatus) {
                // 到达生产工作台
                cmd.buy = true;
                wb[wb_id].reachable = true;     // 该生产工作台可达
                releaseLock(curTask.destCo);
                taskQueue.pop();                
                // clock_t start = clock();
                
                while (1) {
                    dijkstra(rtIdx, location, curMission.endIndex, wb[curMission.endIndex].location);
                    if (pathLength[rtIdx][curMission.endIndex] < 0) {
                        //TODO等待，可能需要移出当前函数
                        break;
                    }
                    if (compress(rtIdx, location, curMission.endIndex, wb[curMission.endIndex].location, 0, 1)) break;
                }
                // clock_t end = clock();
                // cerr << "Frame: " << frameID << " dijkstra one workbench cost" << end-start << endl;
            }
            if (curTask.sell) {
                // 达到消耗工作台
                cmd.sell = true;
                releaseLock(curTask.destCo);
                taskQueue.pop();
            }              
        }
        
    }
}

// 检查任务队列情况
void robot::checkTask() {
    if (taskQueue.empty()) {
        // 分配新任务
        std::vector<mission> msNode; // 任务节点
        // clock_t start = clock();
        dijkstra(rtIdx, location);
        // clock_t end = clock();
        // cerr << "Frame: " << frameID << " dijkstra all workbench cost" << end-start << endl;
        findMission(msNode, location, lsp);
        
        bool success = false;
        for (int i = 0; i < msNode.size(); ++i) {
            mission selected = msNode[i];
            // 预计到达生产工作台时已有产品生成且预计任务能在第9000帧之前完成才接单
            // cerr << "robot" << rtIdx << ": " << frameID << "   " << selected.estFrame + frameID << endl;
            #ifdef ESTIMATE
            if ((wb[selected.startIndex].pstatus || selected.estFrame >= wb[selected.startIndex].rtime) && (selected.estFrame + frameID < 9000)) {
            #else
            if (wb[selected.startIndex].pstatus && (selected.estFrame + frameID < 15000)) {
            #endif
                // cerr << "new Mission: Frame" << frameID << ":(robot" << rtIdx << ") " << selected.startIndex << "->" << selected.endIndex << endl;
                curMission = selected;
                if (compress(rtIdx, location, curMission.startIndex, wb[curMission.startIndex].location, 1, 0)) {
                    wb[selected.startIndex].reachable = false;    // 该生产工作台不可达
                    wb[curMission.endIndex].setProType(curMission.proType);
                    success = true;
                    break;
                }
            }
        }
        if (!success) {
            if (K==25 || K == 18) setSpeed(curFlow.preDestion[this->rtIdx]);
            return;
        }
    }
    if (haveTemDest) {
        // 前往临时目的地
        setSpeed(temDest);
    }
    else {
        // 执行当前任务，前往目的地
        curTask = taskQueue.front();
        setSpeed(curTask.destCo);
    }
}

// 速度过低时用朝向来为其赋一个明确速度
void robot::checkSpeed() {
    if (fabs(lsp.x) < 0.001 && fabs(lsp.y) < 0.001) {
        lsp.x += 0.01*cos(toward);
        lsp.y += 0.01*sin(toward);
    }
}

bool cmp (mission& a, mission& b) {
    return a.v > b.v;
}

void robot::findMission(std::vector<mission>& msNode, coordinate& rtCo, vec& lsp) {
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        // 寻找有现成产品或正在生产中的可达生产工作台
        #ifdef ESTIMATE
        if (wb[wbIdx].reachable && pathLength[rtIdx][wbIdx]>=0 && (wb[wbIdx].pstatus || wb[wbIdx].rtime>=0)) {
        #else 
        if (wb[wbIdx].reachable && pathLength[rtIdx][wbIdx]>=0 && wb[wbIdx].pstatus) {
        #endif
            int proType = wb[wbIdx].type;
            // 遍历收购方
            for (auto buyWbIdx: type2BuyIndex[proType]) {
                if (pathLength[rtIdx][buyWbIdx]<0) continue;
                // 收购方是8或9号工作台，或者，对应原材料格为空
                if (wb[buyWbIdx].type > 7 || !wb[buyWbIdx].checkHaveProType(proType)) {
                    // 此时从 wbIdx 到 buyWbIdx 是一个潜在任务
                    mission pot = mission(wbIdx, buyWbIdx, proType);
                    pot.countValue(rtIdx, proType, lsp);
                    if (K == 18 && rtIdx < 2 && wb[buyWbIdx].type == 4) {
                        pot.v *= 2;
                    }
                    msNode.push_back(pot);
                } 
            }
        }
    }
    sort(msNode.begin(), msNode.end(), cmp);
}
