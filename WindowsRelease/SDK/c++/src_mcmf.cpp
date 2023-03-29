/*** 
 * @Author: Xzh
 * @Date: 2023-03-20 22:55:25
 * @LastEditTime: 2023-03-27 15:49:26
 * @LastEditors: Xzh
 * @Description: 
 *      引入最小费用最大流进行全局任务规划，优化任务分配
 */

#include "inc_codecraft2023.hpp"


void mcmf::init(){
    S = getNode(), T = getNode();

    for (int i = 1; i < 4; i++) {
        curSize[i] = poolSize;
    }
    for (int i = 4; i < 7; i++) {
        curSize[i] = poolSize / 2;
    }
    curSize[7] = poolSize / 6;

    int dx[4] = {-1,-1,1,1};
    int dy[4] = {-1,1,-1,1};
    int ddis = 25/2;

    for (int i = 0; i < N; i++) {
        robotId[i] = getNode(),addEdge(S,robotId[i],0,1);
        preDestion[i].set(dx[i] * ddis + 25, dy[i] * ddis + 25);
    }
    for (int i = 0; i < K; i++) {
        workbenchId[i] = -1;
        for(int j = 0; j < 3; j++) workbenchProductId[i][j] = -1;
    }


    // 为原料格加点加边+维护index数组
    auto setID = [&](int &pst,int wbIdx,int cap = OFFSET){
        pst = getNode();
        addEdge(pst,T,0,cap);
        ProductId2Workbench[pst] = wbIdx;
    };
    // 建立原料格对汇点的边
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 4:
        case 5:
        case 6:
        case 7:
            setID(workbenchProductId[wbIdx][0],wbIdx,1);
            setID(workbenchProductId[wbIdx][1],wbIdx,1);
            break;
        case 8:
        case 9:
            setID(workbenchProductId[wbIdx][0],wbIdx);
            break;
        default:
            break;
        }
        if (wb[wbIdx].type == 7) {
            setID(workbenchProductId[wbIdx][2],wbIdx,1);
        }
    }
    // 建立产品型号到收购方原材料格id的映射
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 4:
            produce2sell[1].insert(workbenchProductId[wbIdx][0]);
            produce2sell[2].insert(workbenchProductId[wbIdx][1]);
            break;
        case 5:
            produce2sell[1].insert(workbenchProductId[wbIdx][0]);
            produce2sell[3].insert(workbenchProductId[wbIdx][1]);
            break;
        case 6:
            produce2sell[2].insert(workbenchProductId[wbIdx][0]);
            produce2sell[3].insert(workbenchProductId[wbIdx][1]);
            break;
        case 7:
            produce2sell[4].insert(workbenchProductId[wbIdx][0]);
            produce2sell[5].insert(workbenchProductId[wbIdx][1]);
            produce2sell[6].insert(workbenchProductId[wbIdx][2]);
            break;
        case 8:
            produce2sell[7].insert(workbenchProductId[wbIdx][0]);
            break;
        case 9:
            for (int i = 1; i <= 7; ++i) {
                produce2sell[i].insert(workbenchProductId[wbIdx][0]);
            }
            break;
        default:
            break;
        }
    }

    // 初始化空闲产品节点池
    if (cnt&1) getNode();
    for (int type = 1; type < 8; ++type){
        for (int cur = 0,size = curSize[type]; cur < size; ++cur){
            pool[type][cur] = getNode();
            int codNode = getNode(), nodeId = codNode ^ 1;
            for (int i = 0; i < N; i++) addEdge(robotId[i],nodeId,0,0);
            addEdge(nodeId,codNode,0,1);
            for (auto id : produce2sell[type]) addEdge(codNode,id,0,0);
        }
    }
}

//静态价值参数调整

double mcmf::countvv(int proType,int endIndex) {
    double vv = profitAndTime[proType].first.first; // 出售收入
    double nextVv = profitAndTime[wb[endIndex].type].first.first - profitAndTime[wb[endIndex].type].first.second;
    // 考虑剩余原材料格对价值的影响，目标工作台的剩余材料格越少越重视
    if (wb[endIndex].type > 7) {
        vv *= 0.8;
        if (K== 18) vv *= 5;
    } else if (wb[endIndex].type == 7) {
        vv += para4*nextVv/(4-wb[endIndex].rawMaterNum());
        if (K== 18) vv *= 3;
    } else if (wb[endIndex].type > 3) {
        vv += para4*nextVv/(3-wb[endIndex].rawMaterNum());
    }
    
    if (wb[endIndex].type == 4) {
        vv *= (1+max(0, min(totalSellNum[5], totalSellNum[6]) - totalSellNum[4]) * 2);
        if (K== 18 && frameID <= 5000) vv*=10;
    } else if (wb[endIndex].type == 5) {
        vv *= (1+max(0, min(totalSellNum[4], totalSellNum[6]) - totalSellNum[5]) * 2);
        if (K== 18 && frameID <= 5000) vv*=1.23 + (frameID <= 1200)*2.86;
    } else if (wb[endIndex].type == 6) {
        vv *= (1+max(0, min(totalSellNum[5], totalSellNum[4]) - totalSellNum[6]) * 2);
    }
    return vv;
}

double mcmf::countValue(int proType,int startIndex,int endIndex) {
    coordinate s = wb[startIndex].location;
    coordinate e = wb[endIndex].location;
    double dd = dis(s, e);                  // 机器人从起点到终点的距离
    double tt = dd/6 + 1;
    double vv = countvv(proType,endIndex); // 出售收入
   
    return  - ( para2 * vv + para1 * tt) + OFFSET;
}

//TODO is it can merge with above function
double mcmf::countSellValue(int proType,int rtIdx,int endIndex){
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    
    double dd = dis(s, e);                      // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);              // 起点到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e);   // 任务所需转动角度和
    double tt = dd/6 + rr/PI + 1;
    double vv = countvv(proType,endIndex); // 出售收入
    
    
    int nextPro = wb[endIndex].type;
    if (K== 25 && rtIdx == 1 && nextPro == 6)  vv *= 4.5;
    // if (rtIdx == 2 && nextPro == 6)  vv *= 3;
    return  - ( para2 * vv + para1 * tt) + OFFSET;
}

double mcmf::countBuyValue(int proType,int rtIdx,int endIndex) {
    if (proType != 0) return INF*2;
    coordinate s = rt[rtIdx].location;
    coordinate e = wb[endIndex].location;
    int left = leftTime[workbenchId[endIndex]]; // 剩余生产时间
    double dd = dis(s, e);                    // 机器人到终点的距离
    vec s2e(e.x - s.x, e.y - s.y);            // 机器人到终点的向量
    double rr = cntAngle(rt[rtIdx].lsp, s2e); // 任务所需转动角度和
    double tt = dd/6 + rr/PI + 1;
    double estFrame = tt * 50;
    // double vv = -profitAndTime[proType].first.second; // 已购入支出
    int nextPro = wb[endIndex].type;
    double vv = -profitAndTime[nextPro].first.second; // 先预计购入支出
    return  -(para2 * vv + para1 * tt) + OFFSET+ max(0.0, left - estFrame) * INF;
}


void mcmf::allocateNode(int wbIdx){
    if (workbenchId[wbIdx] == -1) {
        int id ,type = wb[wbIdx].type;
        id = workbenchId[wbIdx] = pool[type][--curSize[type]];

        ProductId2Workbench[id] = wbIdx;
        ProductId2Workbench[id ^ 1] = wbIdx;
        
        for (int i = 0; i < N; i++) {
            edge &tmp = G[id][i];
            tmp.cap = 0;
            G[tmp.to][tmp.rev].cap = 1;
        }
        G[id][N].cap = 1;
        id ^= 1;
        G[id][0].cap = 0;

        for (int index = 1,size = G[id].size(); index < size; index++) {
            edge &tmp = G[id][index];
            int towbIdx = ProductId2Workbench[tmp.to];
            tmp.cap = 1, G[tmp.to][tmp.rev].cap = 0;
            tmp.cost = countValue(type,wbIdx,towbIdx);
            G[tmp.to][tmp.rev].cost = -tmp.cost;
        }
    }
    int id = workbenchId[wbIdx];
    int tmpTime = wb[wbIdx].rtime;
    if (tmpTime < 0 || wb[wbIdx].pstatus) tmpTime = 0;
    leftTime[id] = leftTime[id ^ 1] = tmpTime;
}

int mcmf::spfa(){
    fill(shortDis, shortDis + cnt, INF);
    memset(vis,0,sizeof(int)*(cnt));
    memset(cut,0,sizeof(int)*(cnt));
    deque<int> q;
    int k = cnt;

    vis[S] = 1,shortDis[S] = 0;
    q.push_back(S);

    while(q.size()){
        int n = q.front();q.pop_front();
        double d = shortDis[n];
        vis[n] = 0;

        for (int i = 0, size = G[n].size(); i < size; i++) {
            const edge& e = G[n][i];
            int to = e.to;

            if (e.cap > 0 && shortDis[to] > d + e.cost + eps) {
                shortDis[to] = d + e.cost + eps,cut[to] = cut[n] + 1,pre[to] = n,pe[to] = i;
                if(cut[to] > k-1) return -1;
                if(!vis[to])  vis[to] = 1,(q.size()&&shortDis[to] < shortDis[q.front()] - eps)?q.push_front(to):q.push_back(to);//slf优化
            }
        }
    }
    return cut[T];
}

int mcmf::solve() {
    if(spfa() < 0) {
        cerr << "fatal mcmf error!!!!" << endl;
        return -1;
    }
    bufCur = 0;
    flow = 0;
    while(spfa() > 0){
        int newflow = OFFSET,index = 0;
        for(int x = T; x != S; x = pre[x])  newflow = min(newflow,G[pre[x]][pe[x]].cap);
        for(int x = T; x != S; x = pre[x])  {
            stateBuf[bufCur][index][0] = pre[x],stateBuf[bufCur][index++][1] = pe[x];
            edge&ed = G[pre[x]][pe[x]];
            ed.cap -= newflow;
            G[ed.to][ed.rev].cap += newflow;
        }
        flow += newflow;
        ++bufCur;
    }
    return 1;
}

void mcmf::showNodeEdge(int id, int condition) {
    if (condition) {
        fprintf(stderr,"\n");
        for (int i = 0,size = G[id].size(); i < size; i++) {
            edge&ed = G[id][i];
            if (G[ed.to][ed.rev].cap || 1) {
                fprintf(stderr," %d %d %d -------> %d %d\n",id,i,ed.cap,ed.to,G[ed.to][ed.rev].cap);
            }
        }
        fprintf(stderr,"\n");
    }
}

void mcmf::showFlow(int condition,int detailed) {
    if (condition) {
        fprintf(stderr,"frameID : %d \n",frameID);
        for (int i = 0; i < bufCur; i++){
            int index = 0,pe,pv;
            fprintf(stderr,"%d ",T);
            do {
                pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
                ++index;
                fprintf(stderr,"<------ %d ",pv);
            }while(pv != S);
            fprintf(stderr,"\n");
        }
        fprintf(stderr,"\n");
        if (detailed) {
            fprintf(stderr,"\n");
            for (int i = 0; i < bufCur; i++){
                int index = 0,pe,pv;
                do {
                    pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
                    edge &ed = G[pv][pe];
                    fprintf(stderr,"%d %d %f-----> %d %d\n",pv,ed.cap,ed.to,ed.cost,G[ed.to][ed.rev].cap);
                    ++index;
                }while(pv != S);
                fprintf(stderr,"\n");
            }
            fprintf(stderr,"\n");
        }
    }

}

void mcmf::resetCap(){
    for (int i = 0; i < bufCur; i++){
        int index = 0,pe,pv;
        do {
            pv = stateBuf[i][index][0],pe = stateBuf[i][index][1];
            ++index;
            edge&ed = G[pv][pe];
            ed.cap += 1;
            G[ed.to][ed.rev].cap -= 1;
        }while(pv != S);
    }
}

void mcmf::releaseNode(int id,int type){
    for(edge&ed:G[id]){
        ed.cap = G[ed.to][ed.rev].cap = 0;
    }
    pool[type][curSize[type]++] = id;
    id ^= 1;
    for(edge&ed:G[id]){
        ed.cap = G[ed.to][ed.rev].cap = 0;
    }
}

void mcmf::lockNode(int rtIdx,int wbIdx){
    rt[rtIdx].nodeId = workbenchId[wbIdx];
    workbenchId[wbIdx] = -1;
}

void mcmf::adjustEdge(int rtIdx){
    int id = robotId[rtIdx];
    edge&tmp = G[id][0]; // robot to S
    tmp.cap = 0,G[S][tmp.rev].cap = 1;

    int nodeId = rt[rtIdx].nodeId,type = wb[ProductId2Workbench[nodeId]].type;
    for (int i = 1,size = G[id].size(); i < size; i++) {
        edge&ed = G[id][i];
        if (ed.to != nodeId) {
            if (!ed.cap) continue;
            ed.cost = countBuyValue(type,rtIdx,ProductId2Workbench[ed.to]);
            G[ed.to][ed.rev].cost = -ed.cost;
            // if (nodeId < 0) ed.cap = 1,G[tmp.to][tmp.rev].cap = 0;
        } else ed.cost = 0,G[ed.to][ed.rev].cost = -ed.cost;
    }

    if (~nodeId) {
        for (int i = 0; i < N; i++) {
            edge &tmp = G[nodeId][i];
            tmp.cap = 0;
            G[tmp.to][tmp.rev].cap = id == tmp.to;
        }

        G[nodeId][N].cap = 1;
        nodeId ^= 1;
        G[nodeId][0].cap = 0;
        
        for (int index = 1,size = G[nodeId].size(); index < size; index++) {
            edge &tmp = G[nodeId][index];
            int towbIdx = ProductId2Workbench[tmp.to];
            tmp.cap = 1, G[tmp.to][tmp.rev].cap = 0;
            tmp.cost = countSellValue(type,rtIdx,towbIdx);
            G[tmp.to][tmp.rev].cost = -tmp.cost;
        }
    } 
}

/**
 *return value : 
 *   0 when a is normal value;
 *   1 when a is NaN;
 *   0 when a is OFFSET;
 */
int mcmf::checkVaild(double a) {
    if (isnan(a)) {
        return 1;
    } else if (isinf(a)) {
        return 2;
    } else {
        return 0;
    }
}

void mcmf::adjustTask(int rtIdx){
    int id = robotId[rtIdx];

    rt[rtIdx].curTask.destId = -1;
    int nextId = -1,nodeId = rt[rtIdx].nodeId;

    if (~nodeId) {
        if (G[nodeId][rtIdx].cap) {
            nodeId ^= 1;
            for (int index = 1,size = G[nodeId].size(); index < size; index++) {
                edge &ed = G[nodeId][index];
                // TODO can first check original dest
                if (G[ed.to][ed.rev].cap) {
                    int wbIdx = ProductId2Workbench[ed.to];
                    rt[rtIdx].curTask = task(wb[wbIdx].location, wbIdx,0,1);
                    break;
                }
            }
        } else {
            cerr << "destroyed " <<rtIdx << "  curframeID " <<frameID<< endl;
            int debugLevel = 1;    // 调试级别 0不输出报错信息,1表示简略的报错信息，2是详细的报错信息。
            int errorType = 0;

            if (debugLevel) {
                auto check = [&](int id) -> int {
                    int flag = 0;
                    for (int i = 0,size = G[id].size(); i < size; i++) {
                        edge&ed = G[id][i];
                        if (G[ed.to][ed.rev].cap || ed.cap) {
                            if (debugLevel > 1)
                            fprintf(stderr,"%d %d %d -------> %d %d %f %f\n",id,i,ed.cap,ed.to,G[ed.to][ed.rev].cap,ed.cost,G[ed.to][ed.rev].cost);
                            flag |= checkVaild(ed.cost);
                        }
                    }
                    if (debugLevel > 1) cerr << endl;
                    errorType |= flag;
                    return flag;
                };

                auto printError = [&](const char *pre,int flag) {
                    if (flag & 1) fprintf(stderr,"%s   NaN checked\n",pre);
                    if (flag & 2) fprintf(stderr,"%s   OFFSETchecked\n",pre);       
                    if (flag & 4) fprintf(stderr,"%s   Unknown Error\n",pre);       
                };

                printError("countBuyValue",check(id));

                cerr << "nodeId "<<nodeId << endl;
                check(nodeId);

                int tmpId = nodeId ^ 1;
                printError("countSellValue",check(tmpId));


                std::vector<int> de;
                for (int i = 0,size = G[tmpId].size(); i < size; i++) {
                    edge&ed = G[tmpId][i];
                    if (ed.cap) de.push_back(ed.to);
                }

                for (auto id : de) {
                    printError("countSellValue | countValue",check(id));
                }

                if (!errorType) printError("",4);
                // showNodeEdge(T);
                
                // for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
                //     int rstatus = wb[wbIdx].rstatus;
                //     fprintf(stderr," wbIdx : %d  rstatus : %d\n", wbIdx, rstatus);
                // }

                showFlow();
                cerr << endl;
            }
            
            // TODO if this situation happen,this solution
        }
    } else {
        for (int i = 1,size = G[id].size(); i < size; i++) {
            edge&ed = G[id][i];
            if (G[ed.to][ed.rev].cap) {
                int wbIdx = ProductId2Workbench[ed.to];
                rt[rtIdx].curTask = task(wb[wbIdx].location, wbIdx,1,0);
                break;
            }
        }
    }
}

void mcmf::checkDest(int rtIdx) {
    robot &bot = rt[rtIdx];    
    if (bot.haveTemDest) { 
        // 检查是否到达临时目的地附近
        if (dis(bot.temDest, bot.location) < 0.5) {
            // 视为到达
            bot.haveTemDest = false;
        }
    } else {
        if (bot.curTask.destId != -1) {
            if (bot.wb_id == bot.curTask.destId) {
                // 到达当前工作目的地，交付工作
                bot.cmd.sell = bot.curTask.sell;
                if (!leftTime[workbenchId[bot.wb_id]]) bot.cmd.buy = bot.curTask.buy;
                
                if (bot.cmd.sell) {
                    if (wb[bot.wb_id].type < 8) {
                        int index = 0;
                        switch (wb[bot.wb_id].type) {
                        case 4:
                            index = bot.pd_id == 2;
                            break;
                        case 5:
                        case 6:
                            index = bot.pd_id == 3;
                            break;
                        case 7:
                            index = bot.pd_id - 4;
                            break;
                        default:
                            break;
                        }
                        int id = workbenchProductId[bot.wb_id][index];
                        setEdgeCap(id,0,0);
                    }
                    releaseNode(bot.nodeId,bot.pd_id); 
                    bot.nodeId = -1;
                } else {
                    if (!leftTime[workbenchId[bot.wb_id]]) {
                        // TODO: assume robot must buy material here,is it not realistic
                        lockNode(rtIdx,bot.wb_id);
                    }
                }
                bot.curTask.destId = -1;
            }
        }
    }
}

void mcmf::switcher() {
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        robot &bot = rt[rtIdx];
        while (!bot.taskQueue.empty()) {
            bot.taskQueue.pop();
        }
        if (bot.nodeId != -1) {
            bot.curMission.set(0, bot.curTask.destId, wb[ProductId2Workbench[bot.nodeId]].type);
            bot.taskQueue.push(bot.curTask);
        }
    }
}

void mcmf::adjustEdge() {
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        if (workbenchId[wbIdx] != -1) {
            int id = workbenchId[wbIdx] ^ 1,type = wb[wbIdx].type;
            for (int index = 1,size = G[id].size(); index < size; index++) {
                edge &tmp = G[id][index];
                int towbIdx = ProductId2Workbench[tmp.to];
                tmp.cost = countValue(type,wbIdx,towbIdx);
                G[tmp.to][tmp.rev].cost = -tmp.cost;
            }
        }
    }
}

void mcmf::solution() {
    // 检查工作台状态
    //TODO parallel
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        if (wb[wbIdx].pstatus || wb[wbIdx].rtime >= 0) {
            allocateNode(wbIdx);
        }
        int rstatus = wb[wbIdx].rstatus;
        auto testAndSetEdgeCap = [&](int bit,int index){
            setEdgeCap(workbenchProductId[wbIdx][index],0, !((rstatus >> bit) & 1));
        };
        switch (wb[wbIdx].type) {
        case 4:
            testAndSetEdgeCap(1,0);
            testAndSetEdgeCap(2,1);
            break;
        case 5:
            testAndSetEdgeCap(1,0);
            testAndSetEdgeCap(3,1);
            break;
        case 6:
            testAndSetEdgeCap(2,0);
            testAndSetEdgeCap(3,1);
            break;
        case 7:
            testAndSetEdgeCap(4,0);
            testAndSetEdgeCap(5,1);
            testAndSetEdgeCap(6,2);
            break;
        default:
            break;
        }
    }

    //TODO parallel
    // 检查机器人运动状态
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        if (rt[rtIdx].holdTime) --rt[rtIdx].holdTime;
        rt[rtIdx].cmd.clean(); // 清除之前指令设置
        rt[rtIdx].checkSpeed();// 保证速度非0
        checkDest(rtIdx);// 检查是否到达目的地
    }
    adjustEdge();// 调整与工作台相连的边权网络

    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        adjustEdge(rtIdx);// 调整网络
    }
    // 指令规划
    solve(); // 费用流运行
    
    auto setDest = [&](int rtIdx){
        if (rt[rtIdx].haveTemDest) {
            rt[rtIdx].setSpeed(rt[rtIdx].temDest);
        } else if(rt[rtIdx].curTask.destId != -1) {
            rt[rtIdx].setSpeed(rt[rtIdx].curTask.destCo);
        } else rt[rtIdx].setSpeed(preDestion[rtIdx]);
    };
    //TODO parallel
    // 检查调整机器人任务执行
    for (int rtIdx = 0; rtIdx < N; ++rtIdx) {
        adjustTask(rtIdx);// 调整任务
        setDest(rtIdx);
    }

    // 碰撞避免
    collitionAvoidance();

    auto printError = [&]() {
        if (flow != N && frameID >= 50) {
            fprintf(stderr, "flow Error curframeID : %d\n\n",frameID);
        }
    };

    // showFlow(frameID >= 4640 && frameID <= 4660);
    // showNodeEdge(T);
    // printError();

    resetCap();
    return;
}
