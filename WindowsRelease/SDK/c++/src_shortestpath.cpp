#include "inc_shortestpath.hpp"

size_t coordinate2_hash::operator()(const coordinate2& c) const {
    return (c.x<<8) | c.y;
}

// 所有工作台的离散坐标
unordered_map<coordinate2, int, coordinate2_hash> workbenchCoordinate;
// 上一次调用dijkstra时机器人的离散坐标
coordinate2 robotCoordinate[ROBOT_SIZE];

// 从机器人i出发的单源最短路径
// rtPrecessor存储了各离散坐标最短路上的前驱
// rtPathLength存储了从源点到该坐标的距离
coordinate2 rtPrecessor[ROBOT_SIZE][MAP_SIZE][MAP_SIZE];   
double rtPathLength[ROBOT_SIZE][WORKBENCH_SIZE]; 

// 从工作台i出发的单源最短路径
// wbPrecessor存储了各离散坐标最短路上的前驱
// wbPathLength存储了从源点到该坐标的距离
coordinate2 wbPrecessor[WORKBENCH_SIZE][MAP_SIZE][MAP_SIZE];   
double wbPathLength[WORKBENCH_SIZE][WORKBENCH_SIZE]; 

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
                posiWeight[i - 1][j] += 20.0; posiWeight[i + 1][j] += 20.0;
                posiWeight[i][j - 1] += 20.0; posiWeight[i][j + 1] += 20.0;
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
    for (int k = 0; k < K; ++k) {
        coordinate2 wbLoca(wb[k].location);
        posiWeight[wbLoca.x][wbLoca.y] = 1;
    }
}

// 预处理，对点的可达性做判断
void initAccessibility() {
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            if (resolve_plat[i+1][j+1] == '#')  continue;
            if (posiWeight[i][j] < 1.1) continue;

            coordinate cur = pointCorrection(coordinate2(i, j));
            double distance;
            const int bias = 2;
            for (int di = -bias; di <= bias; ++di) {
                if (i+di+1 < 0 || i+di+1 > MAP_SIZE+1) continue;
                for (int dj = -bias; dj <= bias; ++dj) {
                    if (!di && !dj) continue;
                    if (j+dj+1 < 0 || j+dj+1 > MAP_SIZE+1) continue;
                    if (resolve_plat[i+di+1][j+dj+1] != '#') continue;
                    double delta_d = sqrt(di*di + dj*dj)/max(fabs(di), fabs(dj));
                    coordinate wall = coordinate2(i+di, j+dj);
                    distance = dis(cur, wall);
                    if (distance <= 0.45 + 0.25*delta_d) {
                        resolve_plat[i+1][j+1] = '1';
                        di = bias + 1;
                        break;
                    }
                    else if (distance <= 0.53 + 0.25*delta_d) {
                        resolve_plat[i+1][j+1] = '3';
                    }
                }
            }
        }
    }

    for (int k = 0; k < K; ++k) {
        coordinate2 c = wb[k].location;        
        int i = c.x, j = c.y;
        if (resolve_plat[i+1][j+1] == '1') {
            for (int di = -1; di <= 1; ++di) {
                if (i+di+1 < 0 || i+di+1 > MAP_SIZE+1) continue;
                for (int dj = -1; dj <= 1; ++dj) {
                    if (!di && !dj) continue;
                    if (j+dj+1 < 0 || j+dj+1 > MAP_SIZE+1) continue;
                    if (resolve_plat[i+di+1][j+dj+1] != '#' && resolve_plat[i+di+1][j+dj+1] != '1') {
                        resolve_plat[i+1][j+1] = '3';
                        di = 2;
                        break;
                    }             
                }
            }
        }
    }
    
}

// 预处理，计算从机器人及工作台到所有工作台的最短路
void initShortestPath(const coordinate2* oricoordinate) {
    // 机器人到所有位置不可达
    for (int i = 0; i < ROBOT_SIZE; i++) {
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            rtPathLength[i][j] = inf;
        }
    }
    // 调用dijkstra计算最短路
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        dijkstra(i, oricoordinate[i], true);
    }

    // 机器人到所有位置不可达
    for (int i = 0; i < WORKBENCH_SIZE; i++) {
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            wbPathLength[i][j] = inf;
        }
    }
    // 调用dijkstra计算最短路
    for (auto it = workbenchCoordinate.begin(); it != workbenchCoordinate.end(); ++it) {
        dijkstra(it->second, it->first, false);
    }
}

// 计算从idx号机器人或idx号工作台到所有工作台的最短路（用于寻找到生产工作台的最短路，因此不携带物品）
// 当flag为true时，表示源点为机器人；当flag为false时，表示源点为工作台
void dijkstra(int idx, coordinate2 src, bool flag) {
    // 当前位置已搜索过
    if (flag) {
        if (robotCoordinate[idx] == src) return;
        robotCoordinate[idx] = src;
        // 原最短路无效
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            rtPathLength[idx][j] = inf;
        }
    }
    // cerr << "enter buy dijkstra: Frame" << frameID << " robot" << rtidx << endl;

    bool visited[MAP_SIZE][MAP_SIZE] = {0};
    priority_queue<node, vector<node>, greater<node>> q;
    coordinate2 (&precessor)[MAP_SIZE][MAP_SIZE] = flag ? rtPrecessor[idx] : wbPrecessor[idx];
    double (&pathLength)[WORKBENCH_SIZE] = flag ? rtPathLength[idx] : wbPathLength[idx];

    q.push(node(0, src));
    visited[src.x][src.y] = true;
    int findk = flag ? 0 : 1;
    while (!q.empty()) {
        int x = q.top().coor.x;
        int y = q.top().coor.y;
        double dis = q.top().distance;
        q.pop();

        for (int i = x-1; i <= x+1; ++i) {
            if (i < 0 || i >= MAP_SIZE) continue;
            for (int j = y-1; j <= y+1; ++j) {
                if (j < 0 || j >= MAP_SIZE) continue;
                if (resolve_plat[i+1][j+1] == '#') continue;
                if (resolve_plat[i+1][j+1] == '1') continue;
                if (!flag && resolve_plat[i+1][j+1] == '3') continue;
                if (flag && !pathlock_isReachable(idx,i,j)) continue;
                if (visited[i][j])  continue;
                precessor[i][j].set(x, y);
                coordinate2 dest(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[i][j]: dis+dis2*posiWeight[i][j];
                if (workbenchCoordinate.count(dest)) {
                    // 当前坐标有工作台，更新最短路
                    ++findk;
                    pathLength[workbenchCoordinate[dest]] = d;
                    if (findk == K) {  
                        // 已找到K个工作台的最短路       
                        // cerr << "success exit buy dijkstra: Frame" << frameID << " robot" << rtidx << endl;               
                        return ;
                    }
                }
                visited[i][j] = true;
                q.push(node(d, dest));           
            }
        }
    }
    
}

// 计算从idx号机器人或idx号工作台到指定工作台的最短路（用于寻找到消耗工作台的最短路，携带了物品）
// 当flag为true时，表示源点为机器人；当flag为false时，表示源点为工作台
void dijkstra(int idx, coordinate2 src, int wbIdx, coordinate2 dest, bool flag) {
    // 当前位置已搜索过
    if (flag) {
        if (robotCoordinate[idx] == src) return;
        robotCoordinate[idx] = src;
        // 原最短路无效
        for (int j = 0; j < WORKBENCH_SIZE; j++) {
            rtPathLength[idx][j] = inf;
        }
    }
    // cerr << "enter sell dijkstra: Frame" << frameID << " robot" << rtidx << " -> " << wbidx << endl;

    bool visited[MAP_SIZE][MAP_SIZE] = {0};
    priority_queue<node, vector<node>, greater<node>> q;   
    coordinate2 (&precessor)[MAP_SIZE][MAP_SIZE] = flag ? rtPrecessor[idx] : wbPrecessor[idx];
    double (&pathLength)[WORKBENCH_SIZE] = flag ? rtPathLength[idx] : wbPathLength[idx];                 

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
                if (resolve_plat[i+1][j+1] == '#') continue;
                if (resolve_plat[i+1][j+1] == '1') continue;
                if (flag && !pathlock_isReachable(idx,i,j)) continue;
                if (resolve_plat[i+1][j+1] == '3') continue;
                if (visited[i][j])  continue;
                precessor[i][j].set(x, y);
                coordinate2 c(i, j);
                double d = (abs(x-i)+abs(y-j)==1) ? dis+dis1*posiWeight[i][j]: dis+dis2*posiWeight[i][j];
                if (c == dest) {
                    // 找到工作台
                    pathLength[wbIdx] = d;
                    // cerr << "success exit sell dijkstra: Frame" << frameID << " robot" << rtidx << " -> " << wbidx << endl;
                    return ;
                }
                visited[i][j] = true;
                q.push(node(d, c));           
            }
        }
    }    
}

// 机器人rtidx调用dijkstra后，将从当前位置到生产工作台，及从生产工作台到消费工作台的最短路进行压缩，并加入任务队列
bool compress(int rtIdx, coordinate2 src, int startIdx, coordinate2 dest1, int endIdx, coordinate2 dest2) {
    robot& r = rt[rtIdx]; 
    while (!r.taskQueue.empty()) r.taskQueue.pop();
    // cerr << "new Task: Frame" << frameID << " robot" << rtIdx << startIdx << " -> " << wbidx << endl;

    stack<coordinate2> s1, s2;
    coordinate2 diff, prediff(-2,-2);
    coordinate2 t, t2 = dest1;

    // 对去往生产工作台的最短路进行压缩   
    s1.push(dest1);
    t = rtPrecessor[rtIdx][dest1.x][dest1.y];    
    while (t != src) {        
        cmpdir(diff, t2, t);
        if (diff == prediff && !pathlock_type(t2)) {
            s1.pop();
        }
        prediff = diff;
        s1.push(t);
        swap(t,t2);
        t = rtPrecessor[rtIdx][t.x][t.y];   
    }
    
    // 对去往消费工作台的最短路进行压缩
    s2.push(dest2);
    t = wbPrecessor[startIdx][dest2.x][dest2.y];
    t2 = dest2, prediff = coordinate2(-2,-2);
    while (t != dest1) {        
        cmpdir(diff, t2, t);
        if (diff == prediff && !pathlock_type(t2)) {
            s2.pop();
        }
        prediff = diff;
        s2.push(t);
        swap(t,t2);
        t = wbPrecessor[startIdx][t.x][t.y];   
    }

    // 加入任务队列
    int flag = 1; 
    auto testAndSet = [&](const coordinate2& c, const int &wbIdx, int buy = 0,int sell = 0){
        if (!pathlock_acquire(rtIdx, c)) {
            flag = 0;
        } else {
            r.taskQueue.push(task(c, wbIdx, buy, sell));
        }
    };

    std::unique_lock<std::mutex> lock(path_mutex);
    while (s1.size() > 1 ) {
        testAndSet(s1.top(), startIdx);
        if (!flag) break;
        s1.pop();
    }
    if (flag) {
        testAndSet(wb[startIdx].location, startIdx, 1, 0);
        if (flag) {
            while (s2.size() > 1) {
                testAndSet(s2.top(), endIdx);
                if (!flag) break;
                s2.pop();
            }
            if (flag) {
                testAndSet(wb[endIdx].location, endIdx, 0, 1);
            }
        }
    }

    if (!flag) {
        // 解锁
        while (!r.taskQueue.empty()) {
            const coordinate2 &c = r.taskQueue.front().destCo;
            pathlock_release(rtIdx, c);
            r.taskQueue.pop();
        }

        // 当前路径无效，需要重新寻找
        // rtPathLength[rtIdx][startIdx] = inf;
        // 下一帧需要重新检测到所有生产工作台的可能路径
        robotCoordinate[rtIdx].set(-1,-1);
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