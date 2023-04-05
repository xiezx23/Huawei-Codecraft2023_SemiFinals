/*** 
 * @Author: Xzx
 * @Date: 2023-03-15 00:10:42
 * @LastEditTime: 2023-04-05 03:21:26
 * @LastEditors: Xzh
 * @Description: 
 */
#include "inc_codecraft2023.hpp"
// #pragma GCC optimize(2)
using namespace std;

int K;                              // 工作台数
int N;                              // 机器人数
int frameID;                        // 当前帧
int curMoney;                       // 当前金钱
robot rt[ROBOT_SIZE];               // 机器人
workbench wb[WORKBENCH_SIZE];       // 工作台
char plat[MAP_SIZE][MAP_SIZE + 2];      // 输入地图
mcmf curFlow;                       // 网络流实例
logInfo dataLog;                    // 日志实例
int totalSellNum[WORKBENCH_SIZE];   // 物品的出售次数

threadPool* tp;     // 线程池

double para1 = 950000;
double para2 = 7;
double para4 = 0.35;

map<int, std::vector<int>> type2BuyIndex;               // 根据产品类型寻找收购方下标
pair<pair<int,int>,int> profitAndTime[WORKBENCH_SIZE];  // 记录收购价、购入价以及生产用时


void init() {    
    tp = new threadPool(4);
    // 读取地图信息
    // 记录机器人的初始坐标
    coordinate2 robotLoc[ROBOT_SIZE];
    K = 0;    
    for(int j = MAP_SIZE-1; j + 1; --j){
        for(int i = 0; i < MAP_SIZE; ++i){
            if(isdigit(plat[i][j])) {
                wb[K].type = plat[i][j] - '0';
                workbenchCoordinate[coordinate2(i, j)] = K; 
                wb[K++].reachable = true;                
            }
            else if(plat[i][j] == 'A') {                
                robotLoc[N++] = coordinate2(i, j);
            }
        }
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        rt[i].rtIdx = i;
    }
    initShortestPath(robotLoc);
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
    // // 预初始化网络流
    // curFlow.init();
}


int main() {
    readPlat();
    initWeight();
    pathlock_init();
    init();
    // printMap();
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

    tp->exit();
    return 0;
}