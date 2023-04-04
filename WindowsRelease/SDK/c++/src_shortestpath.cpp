#include "inc_shortestpath.hpp"

size_t coordinate2_hash::operator()(const coordinate2& c) const {
    return (c.x<<8) | c.y;
}

// 所有工作台的网格化坐标
unordered_map<coordinate2, int, coordinate2_hash> workbenchLoc;
// 机器人i到工作台k的最短路径存储在数组 shortestPath[i][k]中
// 该路径的长度存储于pathLength[i][k]中
// 该路径的节点数存储于pathSize[i][k]中
// 路径以逆序存储！！！（从目的地到源）
coordinate2 shortestPath[ROBOT_SIZE][WORKBENCH_SIZE][MAP_SIZE*MAP_SIZE];
double pathLength[ROBOT_SIZE][WORKBENCH_SIZE];
int pathSize[ROBOT_SIZE][WORKBENCH_SIZE];

// 用于存储最短路上前驱的临时数组
coordinate2 precessor[MAP_SIZE][MAP_SIZE];    

// 水平或直接相邻的距离及对角相邻的距离
const double dis1 = 0.5, dis2 = sqrt(0.5);
const int inf = -1;

// dijkstra算法优先队列中的结构，first为到源节点的距离，second为离散坐标
typedef pair<double, coordinate2> dijkstraNode;
struct cmp {
    inline bool operator()(const dijkstraNode& op1, const dijkstraNode& op2) const {
        return op1.first > op2.first;
    }
};

// 加入位置权重
double posiWeight[MAP_SIZE][MAP_SIZE];
// 对于普通的点，其权重为1，一个墙壁点会对其上下左右的点增加权重2，对斜对角点增加权重1.4
void initWeight() {
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            posiWeight[i][j] = 1.0;
        }
    }
    // for (int i = 0; i < MAP_SIZE; ++i) {
    //     posiWeight[i][0] = posiWeight[i][MAP_SIZE-1] = 10.0;
    //     posiWeight[0][i] = posiWeight[MAP_SIZE-1][i] = 10.0;
    // }
    for (int i = 1; i < MAP_SIZE - 1; ++i) {
        for (int j = 1; j < MAP_SIZE - 1; ++j) {
            if (plat[i][j] == '#') {
                posiWeight[i - 1][j] += 10.0; posiWeight[i + 1][j] += 10.0;
                posiWeight[i][j - 1] += 10.0; posiWeight[i][j + 1] += 10.0;
                posiWeight[i - 1][j + 1] += 7.0; posiWeight[i + 1][j + 1] += 7.0;
                posiWeight[i - 1][j - 1] += 7.0; posiWeight[i + 1][j - 1] += 7.0;
                // for (int x = max(0, i - 2); x < min(100, i + 3); ++x) {
                //     if (j + 2 < 100) posiWeight[x][j + 2] += 1; 
                //     if (j - 2 >= 0) posiWeight[x][j - 2] += 1; 
                // }
                // for (int y = j - 1; y < j + 2; ++y) {
                //     if (i + 2 < 100) posiWeight[i + 2][y] += 1; 
                //     if (i - 2 >= 0) posiWeight[i - 2][y] += 1; 
                // }
            }
        }
    }
}

// 计算从rtidx号机器人到所有工作台的最短路
void dijkstra(int rtidx, coordinate2 src) {
    // 当前位置已搜索过
    if (pathSize[rtidx][0] != inf) return;

    bool visited[MAP_SIZE][MAP_SIZE] = {0};
    priority_queue<node, vector<node>, greater<node>> q;

    q.push(node(0, src));
    visited[src.x][src.y] = true;
    int findk = 0;
    while (!q.empty()) {
        int x = q.top().coor.x;
        int y = q.top().coor.y;
        double dis = q.top().distance;
        q.pop();

        for (int i = x-1; i <= x+1; ++i) {
            if (i < 0 || i >= MAP_SIZE) continue;
            for (int j = y-1; j <= y+1; ++j) {
                if (j < 0 || j >= MAP_SIZE) continue;
                // if (plat[MAP_SIZE-j-1][i] == '#') continue;
                if (resolve_plat[MAP_SIZE-j][i+1] == '#') continue;
                if (resolve_plat[MAP_SIZE-j][i+1] == '1') continue;
                if (visited[i][j])  continue;
                precessor[i][j].set(x, y);
                coordinate2 dest(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[MAP_SIZE-j-1][i]: dis+dis2*posiWeight[MAP_SIZE-j-1][i];
                if (workbenchLoc.count(dest)) {
                    // 当前坐标有工作台，更新最短路
                    ++findk;
                    updatePath(rtidx, src, workbenchLoc[dest], dest, d);
                    if (findk == K) {  
                        // 已找到K个工作台的最短路                      
                        return ;
                    }
                }
                visited[i][j] = true;
                q.push(node(d, dest));           
            }
        }
    }
}

// 计算从rtidx号机器人到指定工作台的最短路（用于寻找到消耗工作台的最短路，携带了物品）
void dijkstra(int rtidx, coordinate2 src, int wbidx, coordinate2 dest) {
    // 当前位置已搜索过
    if (pathSize[rtidx][wbidx] != inf) return;

    bool visited[MAP_SIZE][MAP_SIZE] = {0};
    priority_queue<node, vector<node>, greater<node>> q;                    

    q.push(node(0, src));
    visited[src.x][src.y] = true;
    while (!q.empty()) {
        int x = q.top().coor.x;
        int y = q.top().coor.y;
        double dis = q.top().distance;
        q.pop();

        for (int i = x-1; i <= x+1; ++i) {
            if (i < 0 || i >= MAP_SIZE) continue;;
            for (int j = y-1; j <= y+1; ++j) {
                if (j < 0 || j >= MAP_SIZE) continue;
                // if (plat[MAP_SIZE-j-1][i] == '#') continue;
                if (resolve_plat[MAP_SIZE-j][i+1] == '#') continue;
                if (resolve_plat[MAP_SIZE-j][i+1] == '1') continue;
                // if (isalpha(resolve_plat[MAP_SIZE-j][i+1] )) continue;
                if (visited[i][j])  continue;
                precessor[i][j].set(x, y);
                coordinate2 c(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[MAP_SIZE-j-1][i]: dis+dis2*posiWeight[MAP_SIZE-j-1][i];
                if (c == dest) {
                    // 找到工作台
                    updatePath(rtidx, src, wbidx, dest, d);
                    return ;
                }
                visited[i][j] = true;
                q.push(node(d, c));           
            }
        }
    }    
}

// 根据最短路径前驱表更新最短路
void updatePath(int rtidx, const coordinate2& src, int wbidx, coordinate2& dest, double dis) {
    // 迭代存储最短路上的前驱节点
    pathLength[rtidx][wbidx] = dis;
    pathSize[rtidx][wbidx] = 0;
    while (precessor[dest.x][dest.y] != src) {
        shortestPath[rtidx][wbidx][pathSize[rtidx][wbidx]++] = dest;
        dest = precessor[dest.x][dest.y];
    }
}

// 计算从机器人初始位置到达所有工作台的最短路
void initShortestPath(const coordinate2* oricoordinate) {
    memset(pathSize, inf, sizeof(pathSize));
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        dijkstra(i, oricoordinate[i]);
    }
}

// 机器人rtidx调用dijkstra后，对最短路进行压缩，并将压缩后的最短路加入任务队列中
void compress(int rtidx, int wbidx, bool buy, bool sell) {
    robot& r = rt[rtidx];
    int n = pathSize[rtidx][wbidx];
    if (n < 3) {
        r.taskQueue.push(task(wb[wbidx].location, wbidx, buy, sell));
        return ;
    }

    // 路径压缩   
    stack<coordinate2> s;
    s.push(shortestPath[rtidx][wbidx][0]);
    s.push(shortestPath[rtidx][wbidx][1]);
    coordinate2 diff, prediff;
    cmpdir(prediff, shortestPath[rtidx][wbidx][0], shortestPath[rtidx][wbidx][1]);
    for (int i = 2; i < n; ++i) {
        cmpdir(diff, s.top(), shortestPath[rtidx][wbidx][i]);
        if (diff == prediff) {
            s.pop();
        }
        prediff = diff;
        s.push(shortestPath[rtidx][wbidx][i]);
    }
    
    // 加入任务队列
    // ofstream fout("log.txt", ios_base::app);
    // fout << "new Task: Frame" << frameID << ":(robot" << rtidx << ", " << "work: " << wbidx << ")" << endl;    
    while (s.size() > 1) {
        coordinate c = coordinate(s.top());
        r.taskQueue.push(task(c, wbidx, 0, 0));
        // fout << c.x << ", " << c.y << "(" << s.top().x << ", " << s.top().y << ") -> ";  
        s.pop();
    }
    r.taskQueue.push(task(wb[wbidx].location, wbidx, buy, sell));
    // fout << wb[wbidx].location.x << ", " << wb[wbidx].location.y << endl << endl;  
    // fout.close();

    // 认为机器人位置发生改变，原最短路无效
    memset(pathSize+rtidx, inf, sizeof(int)*WORKBENCH_SIZE);
}

// 比较方向
inline void cmpdir(coordinate2& dir, const coordinate2& c1, const coordinate2& c2) {
    if (c1.x > c2.x) {
        dir.x = 1;
    }
    else if (c1.x == c2.x) {
        dir.x = 0;
    }
    else {
        dir.x = -1;
    }

    if (c1.y > c2.y) {
        dir.y = 1;
    }
    else if (c1.y == c2.y) {
        dir.y = 0;
    }
    else {
        dir.y = -1;
    }
}