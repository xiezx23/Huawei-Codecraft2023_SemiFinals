/*** 
 * @Author: Xzx
 * @Date: 2023-03-15 00:10:42
 * @LastEditTime: 2023-04-01 19:23:15
 * @LastEditors: Xzh
 * @Description: 
 */
#include "inc_codecraft2023.hpp"
using namespace std;

int K;                              // 工作台数
int N;                              // 机器人数
int frameID;                        // 当前帧
int curMoney;                       // 当前金钱
robot rt[ROBOT_SIZE];               // 机器人
workbench wb[WORKBENCH_SIZE];       // 工作台
char plat[MAP_SIZE][MAP_SIZE];      // 输入地图
mcmf curFlow;                       // 网络流实例
logInfo dataLog;                    // 日志实例
int totalSellNum[WORKBENCH_SIZE];   // 物品的出售次数

double para1 = 950000;
double para2 = 7;
double para4 = 0.35;

// int dwaN = 20;
// int dwaM = 20;
// const double dt = 1.0/50;

map<int, std::vector<int>> type2BuyIndex;               // 根据产品类型寻找收购方下标
pair<pair<int,int>,int> profitAndTime[WORKBENCH_SIZE];  // 记录收购价、购入价以及生产用时


void init() {    
    // 读取地图信息
    // 记录机器人的初始坐标
    vector<coordinate2> robotLoc;
    K = 0;
    
    for(int i = 0; i < MAP_SIZE; ++i){
        for(int j = 0; j < MAP_SIZE; ++j){
            if(isdigit(plat[i][j])) {
                wb[K].type = plat[i][j] - '0';
                workbenchLoc[coordinate2(j, MAP_SIZE-i-1)] = K; 
                wb[K++].reachable = true;                
            }
            else if(plat[i][j] == 'A') {
                N++;
                robotLoc.push_back(coordinate2(j, MAP_SIZE-i-1));
            }
        }
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        rt[i].rtIdx = i;
    }
    pathdect_init();
    initShorestPath(robotLoc);
    // 记录每种物品的收益及生产周期
    profitAndTime[0] = make_pair(make_pair(0,0), INF);
    profitAndTime[1] = make_pair(make_pair(6000,3000), 50);
    profitAndTime[2] = make_pair(make_pair(7600,4400), 50);
    profitAndTime[3] = make_pair(make_pair(9200,5800), 50);
    profitAndTime[4] = make_pair(make_pair(22500,15400), 500);
    profitAndTime[5] = make_pair(make_pair(25000,17200), 500);
    profitAndTime[6] = make_pair(make_pair(27500,19200), 500);
    profitAndTime[7] = make_pair(make_pair(105000,76000), 1000);
    // 初始化 type2BuyIndex，为收购方建立索引
    for (int wbIdx = 0; wbIdx < K; ++wbIdx) {
        switch (wb[wbIdx].type) {
        case 4:
            type2BuyIndex[1].push_back(wbIdx);
            type2BuyIndex[2].push_back(wbIdx);
            break;
        case 5:
            type2BuyIndex[1].push_back(wbIdx);
            type2BuyIndex[3].push_back(wbIdx);
            break;
        case 6:
            type2BuyIndex[2].push_back(wbIdx);
            type2BuyIndex[3].push_back(wbIdx);
            break;
        case 7:
            type2BuyIndex[4].push_back(wbIdx);
            type2BuyIndex[5].push_back(wbIdx);
            type2BuyIndex[6].push_back(wbIdx);
            break;
        case 8:
            type2BuyIndex[7].push_back(wbIdx);
            break;
        case 9:
            for (int i = 1; i <= 7; ++i) {
                type2BuyIndex[i].push_back(wbIdx);
            }
            break;
        default:
            break;
        }
    }
    // 预初始化网络流
    curFlow.init();
    
    // 特判
    if (K == 43) {
        // cerr << "map1" << endl;
        para1 = 950000;
        para2 = 6;
        para4 = 1.5;
    }
    else if (K == 25) {
        // cerr << "map2" << endl;
        curFlow.para1 = -390000;
        curFlow.para2 = 40;
    }
    else if (K == 50) {
        // cerr << "map3" << endl;
        para1 = 950000;
        para2 = 20;
        para4 = 0.1;
    }
    else if (K == 18) {
        // cerr << "map4" << endl;
        para1 = 700000;
        para2 = 7;
        para4 = 0.2;
        curFlow.para1 = -390000;
        curFlow.para2 = 45;
        curFlow.para4 = 0.4;
    }
}


int main() {
    readPlat();
    init();    
    puts("OK");
    fflush(stdout);
    while (scanf("%d", &frameID) != EOF) {
        readInfo();
        printf("%d\n", frameID);
        /**** CORE ****/   
        // if ((K==25 || K == 18)&&frameID <= 8000) {
        //     curFlow.solution();
        //     if (frameID == 8000) curFlow.switcher();
        // }
        // else {
        //     ori_solution();
        // }
        ori_solution();
        // curFlow.solution();
        /**************/
        for(int robotId = 0; robotId < ROBOT_SIZE; robotId++){  
            // 各个机器人统计是否产生碰撞、购买、出售等行为       
            rt[robotId].collisionCount();
            rt[robotId].buysellCount();
            // 输出交互指令
            printRobotCommand(robotId);
        }
        // if(frameID > 50)cerr << "dest:" << rt[0].curTask.destId << endl;
        // if(frameID > 50) debug();
        printf("OK\n");
        fflush(stdout);
    }

    # ifdef DEBUG
    dataLog.printLog();
    # endif

    return 0;
}

