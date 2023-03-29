#ifndef MCMF_HPP
#define MCMF_HPP
#include "inc_codecraft2023.hpp"

struct mcmf {
    
    const double eps = 1e-6;
    double para1 = -45000; // 时间相关系数
    double para2 = 5;    // 价值相关系数
    double para4 = 0.3; // 价值期望系数


    int S, T;
    int robotId[ROBOT_SIZE];
    int workbenchProductId[WORKBENCH_SIZE][3]; // 工作台原料格id
    int ProductId2Workbench[maxNode];          // 原料格id转工作台 //TODO:check whether should use map

    int workbenchId[WORKBENCH_SIZE];           // 工作台产品格id
    int cnt;                                        // 节点总数
    int curSize[10];                               // 空闲产品节点数量
    int pool[10][poolSize];                        // 空闲节点池

    std::set<int> produce2sell[10];                 // TODO :whether should move to global

    struct edge {                              // 边
        int rev,to,cap;
        double cost;
        edge(){};
        edge(int to,double cost,int cap,int rev):to(to),cost(cost),cap(cap),rev(rev){}

    };
    std::vector<edge> G[maxNode];            // 邻接表
    void addEdge(int from,int to,double cost,int cap){                // 加边
        G[from].push_back(edge(to,cost,cap,G[to].size()));
        G[to].push_back(edge(from,-cost,0,G[from].size()-1));
    }
    void setEdgeCap(int from,int index,int cap){
        edge &tmp = G[from][index];
        tmp.cap = cap;
        G[tmp.to][tmp.rev].cap = 0;
    }
    void setEdgeCost(int from,int index,double cost){
        edge &tmp = G[from][index];
        tmp.cost = cost;
        G[tmp.to][tmp.rev].cost = -cost;
    }
    

    double shortDis[maxNode];                // 最短路
    int cut[maxNode];                        // 顶点访问次数
    int vis[maxNode];                        // 顶点是否在队列中
    int pre[maxNode];                        // 最短路上的前驱节点
    int pe[maxNode];                         // 最短路上的前驱边
    coordinate preDestion[ROBOT_SIZE];          // 预设坐标
    int leftTime[maxNode];                   // 剩于生产时间
    int stateBuf[ROBOT_SIZE][ROBOT_SIZE * 15][2];// 用于权值回退
    int bufCur = 0;                          // 已使用的Buf数
    int flow = 0;

    
    int getNode(){return cnt++;}
    void init();                        // 在init()后运行

    mcmf():cnt(0){}
    int spfa();                         
    int solve();                        // 基于spfa计算最小费用流

    void releaseNode(int id,int type);           // 将某个节点移入空闲节点池
    void lockNode(int rtIdx,int wbIdx); // 工作台产品被机器人获取
    void allocateNode(int wbIdx);       // 为工作台产品分配节点

    void adjustEdge(int rtIdx);         // 每帧开始时为机器人调整边权
    void adjustTask(int rtIdx);         // 每帧开始时根据费用流为机器人调整任务
    void resetCap();                    // 每帧结束时回退费用流
    void showNodeEdge(int id,int condition = 1); //打印输出某一个节点的所有边
    void showFlow(int condition = 1,int detailed = 0);   // 可视化当前网络流                 
    void solution();
    void checkDest(int rtIdx);
    int checkVaild(double a);           // 检测浮点数类型

    // 以下计算的为代价,越小优先级越大
    // 计算价值函数,计算机器人购买产品后，将其运往出售的开销
    // 参数依次为携带产品类型、起点、终点
    double countValue(int proType,int startIndex,int endIndex);
    double countvv(int proType,int endIndex); //计算出售收入

    // 计算价值函数,计算机器人购买产品后，将其销毁并前往其它工作台的开销
    // 参数依次为携带产品类型(0表示没有)、机器人下标、终点
    double countBuyValue(int proType,int rtIdx,int endIndex);

    // 计算价值函数,计算机器人从当前位置前往对应工作台出售的开销
    // 参数依次为携带产品类型、机器人下标、终点
    double countSellValue(int proType,int rtIdx,int endIndex);

    // 用以兼容在帧开始或调度完成时在网络流和贪心之间切换
    void switcher();

    // 用于调整countBuyvalue边权
    void adjustEdge();
};

extern mcmf curFlow;    // 网络流实例
#endif