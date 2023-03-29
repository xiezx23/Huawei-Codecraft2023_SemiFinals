#include "inc_shorestpath.hpp"

size_t coordinate2_hash::operator()(const coordinate2& c) const {
    std::hash<int> int_hash;
    return int_hash(c.x)^int_hash(c.y);
}

// 记录地图中的障碍物
bool obstacle[MAP_SIZE][MAP_SIZE];
// 所有工作台的网格化坐标
unordered_map<coordinate2, int, coordinate2_hash> workbenchLoc;
// 四维向量，shortestPath[x][y][k][i]表示坐标(x,y)到工作台k的第i个节点的坐标
vector<vector<vector<vector<coordinate2>>>>  shorestPath(MAP_SIZE, vector<vector<vector<coordinate2>>>(MAP_SIZE, vector<vector<coordinate2>>(WORKBENCH_SIZE)));
// 水平或直接相邻的距离及对角相邻的距离
double dis1, dis2;

// 计算从机器人初始位置到达所有工作台的最短路
void initShorestPath(const vector<coordinate2>& oricoordinate) {
    dis1 = 0.5;
    dis2 = sqrt(2*dis1*dis1);
    for (auto coor: oricoordinate) {
        dijkstra(coor);
    }
}

// 根据最短路径前驱表更新最短路
void updatePath(coordinate2 src, coordinate2 dest, const vector<vector<coordinate2>>& precessor) {
    int index = workbenchLoc[dest];
    vector<coordinate2>& v = shorestPath[src.x][src.y][index];
    while (precessor[dest.x][dest.y] != src) {
        v.emplace(v.begin(), dest);
        dest = precessor[dest.x][dest.y];
    }
}

// 计算从src坐标到所有工作台的最短路
void dijkstra(coordinate2 src) {
    vector<vector<coordinate2>> precessor(MAP_SIZE, vector<coordinate2>(MAP_SIZE));
    vector<vector<bool>> visited(MAP_SIZE, vector<bool>(MAP_SIZE, false));
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
            if (i < 0 || i >= MAP_SIZE) continue;;
            for (int j = y-1; j <= y+1; ++j) {
                if (j < 0 || j >= MAP_SIZE) continue;
                if (obstacle[i][j]) continue;
                if (visited[i][j])  continue;
                precessor[i][j].set(x, y);
                coordinate2 c(i, j);
                if (workbenchLoc.count(c)) {
                    ++findk;
                    updatePath(src, c, precessor);
                    if (findk == K) {                        
                        return ;
                    }
                }
                visited[i][j] = true;
                if (abs(x-i)+abs(y-j)==1) {
                    q.push(node(dis+dis1, c));
                }
                else {
                    q.push(node(dis+dis2, c));
                }                
            }
        }
    }
}