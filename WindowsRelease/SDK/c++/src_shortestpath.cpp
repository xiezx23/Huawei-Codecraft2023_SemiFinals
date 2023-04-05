#include "inc_shortestpath.hpp"

size_t coordinate2_hash::operator()(const coordinate2& c) const {
    return (c.x<<8) | c.y;
}

// 所有工作台的离散坐标
unordered_map<coordinate2, int, coordinate2_hash> workbenchLoc;

// 从机器人i出发的单元最短路径
// percessor存储了各离散坐标最短路上的前驱
// pathLength存储了从源点到该坐标的距离
coordinate2 precessor[ROBOT_SIZE][MAP_SIZE][MAP_SIZE];   
double pathLength[ROBOT_SIZE][WORKBENCH_SIZE]; 

// 水平或直接相邻的距离及对角相邻的距离
const double dis1 = 1, dis2 = sqrt(2);

// 不可达标志
const double inf = -1;

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

// 计算从机器人初始位置到达所有工作台的最短路
void initShortestPath(const coordinate2* oricoordinate) {
    // 机器人到所有位置不可达
    for (int i = 0; i < ROBOT_SIZE; i++) {
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            pathLength[i][j] = inf;
        }
    }
    // 调用dijkstra计算最短路
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        dijkstra(i, oricoordinate[i]);
    }
}

// 计算从rtidx号机器人到所有工作台的最短路
void dijkstra(int rtidx, coordinate2 src) {
    // 当前位置已搜索过
    if (pathLength[rtidx][0] >= 0) return;

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
                // if (plat[i][j] == '#') continue;
                if (resolve_plat[i+1][j+1] == '#') continue;
                if (resolve_plat[i+1][j+1] == '1') continue;
                if (!pathlock_isReachable(rtidx,i,j)) continue;
                if (visited[i][j])  continue;
                precessor[rtidx][i][j].set(x, y);
                coordinate2 dest(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[i][j]: dis+dis2*posiWeight[i][j];
                if (workbenchLoc.count(dest)) {
                    // 当前坐标有工作台，更新最短路
                    ++findk;
                    pathLength[rtidx][workbenchLoc[dest]] = d;
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
    if (pathLength[rtidx][wbidx] >= 0) return;

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
                // if (plat[i][j] == '#') continue;
                if (resolve_plat[i+1][j+1] == '#') continue;
                if (resolve_plat[i+1][j+1] == '1') continue;
                if (!pathlock_isReachable(rtidx,i,j)) continue;
                if (visited[i][j])  continue;
                precessor[rtidx][i][j].set(x, y);
                coordinate2 c(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[i][j]: dis+dis2*posiWeight[i][j];
                if (c == dest) {
                    // 找到工作台
                    pathLength[rtidx][wbidx] = d;
                    return ;
                }
                visited[i][j] = true;
                q.push(node(d, c));           
            }
        }
    }    
}

// 机器人rtidx调用dijkstra后，对最短路进行压缩，并将压缩后的最短路加入任务队列中
bool compress(int rtidx, coordinate2 src, int wbidx, coordinate2 dest, bool buy, bool sell) {
    robot& r = rt[rtidx]; 
    while (!r.taskQueue.empty()) r.taskQueue.pop();

    // 压缩路径
    stack<coordinate2> s;
    s.push(dest);
    dest = precessor[rtidx][dest.x][dest.y];
    coordinate2 diff, prediff(-2, -2);
    while (dest != src) {        
        cmpdir(diff, s.top(), dest);
        if (diff == prediff && !pathlock_type(s.top().x, s.top().y)) {
            s.pop();
        }
        prediff = diff;
        s.push(dest);
        dest = precessor[rtidx][dest.x][dest.y];
    }
        
    // 加锁并加入任务队列
    int flag = 1; 
    std::unique_lock<std::mutex> lock(path_mutex);
    while (s.size() > 1) {
        const coordinate2& c = s.top();
        if (!pathlock_acquire(rtidx, c.x, c.y)) {
            flag = 0;
            fprintf(stderr,"fail frameId %d rtIdx:%d wbIdx:%d lockID:%d\n", frameID,rtidx, wbidx, pathlock_type(c.x,c.y));
            break;
        }
        r.taskQueue.push(task(c, wbidx, 0, 0));
        s.pop();
    }
    if (flag) {
        coordinate2 c = wb[wbidx].location;
        if (!pathlock_acquire(rtidx, c.x, c.y)) {
            flag = 0;
            fprintf(stderr,"fail frameId %d rtIdx:%d wbIdx:%d lockID:%d\n", frameID,rtidx, wbidx, pathlock_type(c.x,c.y));
        } else r.taskQueue.push(task(wb[wbidx].location, wbidx, buy, sell));
    }

    if (flag) {

        // 认为机器人位置发生改变，原最短路无效
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            pathLength[rtidx][j] = inf;
        }
    } else {
        // 解锁
        while (!r.taskQueue.empty()) {
            const coordinate2 c = r.taskQueue.front().destCo;
            pathlock_release(rtidx, c.x, c.y);
            r.taskQueue.pop();
        }
        
        // 当前路径无效，需要重新寻找
        pathLength[rtidx][wbidx] = inf;

    }
    return flag;
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