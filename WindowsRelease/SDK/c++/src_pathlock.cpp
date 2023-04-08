#include "inc_pathlock.hpp"

char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
int pathdetect_f[(MAP_SIZE + 2)*(MAP_SIZE + 2)];

int lockID[MAP_SIZE][MAP_SIZE];
int lockCnt = 1;

fhq_treap<pathlock_node> lockStatus_s[MAP_SIZE * MAP_SIZE][4];
fhq_treap<int> lockStatus_e[MAP_SIZE * MAP_SIZE][4];
int lockSize[MAP_SIZE * MAP_SIZE];


std::shared_timed_mutex path_mutex; 

// 并查集维护连通分量
int pathdetect_find(int g){
    return pathdetect_f[g] < 0 ? g : pathdetect_f[g] = pathdetect_find(pathdetect_f[g]);
};

void pathdetect_merge(int a,int b) {
    a = pathdetect_find(a);
    b = pathdetect_find(b);
    if (a != b) {
        pathdetect_f[b] = a;
    }
}


void pathlock_init() {
    lockCnt = 1;
    // 转换地图
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            resolve_plat[i + 1][j + 1] = plat[i][j] != '#'?'.':plat[i][j];
        }
    }
    for (int i = 0; i < MAP_SIZE + 2; i++) {
        resolve_plat[i][0] = resolve_plat[i][MAP_SIZE + 1] = '#';
        resolve_plat[0][i] = resolve_plat[MAP_SIZE + 1][i] = '#';
    }

    for (int i = 0; i < MAP_SIZE + 2; i++) 
            for (int j = 0; j < MAP_SIZE + 2; j++)  
                pathdetect_f[i * (MAP_SIZE + 2) + j] = -1;

    
    // 地图比例尺一格0.5m, 机器人半径0.53m，
    auto rLabel = [](double r, char c){

        auto setC = [&](int i, int j) {
            if (resolve_plat[i][j] == '.') {
                resolve_plat[i][j] = c;
            }
        };

        r += 0.25;
        // 若两个#间距小于2r，则将连线上的.标记成c
        r *= 2;
        int dM = ceil(r/0.5);
        for (int i = 0; i < MAP_SIZE + 2; i++) {
            for (int j = 0; j < MAP_SIZE + 2; j++) {
                if (resolve_plat[i][j] == '#') {
                    for (int di = 0; di <= dM; di++) {
                        if (i + di > MAP_SIZE + 1) break;
                        for (int dj = -dM; dj <= dM; dj++) {
                            if (j + dj > MAP_SIZE + 1) break;
                            if (j + dj < 0) continue;
                            if (!di && !dj) continue;
                            if (resolve_plat[i + di][j + dj] == '#') {
                                if (sqrt(di * di + dj * dj) <= 2 * r) {
                                    if (!di) {
                                        for (int k = j + 1; k < j + dj; k++) {
                                            setC(i,k);
                                        }
                                        for (int k = j + dj; k < j; k++) {
                                            setC(i,k);
                                        }
                                    } else if(!dj) {
                                        for (int k = i + 1; k < i + di; k++) {
                                            setC(k,j);
                                        }
                                    } else if(di == 2 && abs(dj) == 2) {
                                        setC(i + 1, j + dj/2);
                                    } 
                                    else if(di + abs(dj) == 5) {
                                        int sgnj = dj/abs(dj);
                                        if (di == 2) {
                                            setC(i + 1, j + sgnj);
                                            setC(i + 1, j + sgnj * 2);
                                        } else {
                                            setC(i + 1, j + sgnj);
                                            setC(i + 2, j + sgnj);
                                        }
                                    }
                                    else if(di + abs(dj) == 6) {
                                        if (di == abs(dj)){
                                            int sgnj = dj/abs(dj);
                                            setC(i + 1, j + sgnj);
                                            setC(i + 2, j + sgnj);
                                            setC(i + 2, j + 2*sgnj);
                                            setC(i + 1, j + 2*sgnj);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    // 任何机器人无法通过
    rLabel(0.45, '1');    
    // 单行道
    rLabel(0.99, '2');

    // 合并字符为'2'的连通域，并用a-z标记
    for (int i = 1; i <= MAP_SIZE + 1; i++) {
        for (int j = 1; j <= MAP_SIZE + 1; j++) {
            if (resolve_plat[i][j] == '2') {
                int pos = i * (MAP_SIZE + 2) + j;
                int fa = pathdetect_find(pos);
                if (fa == pos) {
                    lockID[i - 1][j - 1] = lockCnt++;
                } else {
                    lockID[i - 1][j - 1] = lockID[fa / (MAP_SIZE + 2) - 1][(fa % (MAP_SIZE + 2))-1];
                }

                if (resolve_plat[i][j + 1] == '2') {
                    pathdetect_merge(pos, pos + 1);
                } 

                if (resolve_plat[i + 1][j] == '2') {
                    pathdetect_merge(pos, pos + (MAP_SIZE + 2));
                    if (resolve_plat[i + 1][j + 1] == '2') {
                        pathdetect_merge(pos, pos + (MAP_SIZE + 2) + 1);
                    }
                    if (resolve_plat[i + 1][j - 1] == '2') {
                        pathdetect_merge(pos, pos + (MAP_SIZE + 2) - 1);
                    }
                }

            }
        }
    }

    // 初始化锁状态
    lockSize[0] = 0x3f3f3f;
    for (int i = 1; i < lockCnt; i++) lockSize[i] = 1;

}

// 输出处理后的地图
void printMap() {
    char ma[256];
    for (int i = 0; i < 26; i++) ma[i] = 'a' + i;
    for (int i = 26; i < 52; i++) ma[i] = 'A' + i - 26;

    // 输出处理后的地图
    for (int j = MAP_SIZE - 1; j + 1; --j) {
        for (int i = 0; i < MAP_SIZE; i++) {
            // fprintf(stderr,"%c", resolve_plat[i+1][j+1]);
            if (lockID[i][j]) fprintf(stderr,"%c", ma[lockID[i][j]%52]);
            else fprintf(stderr,"%c", resolve_plat[i+1][j+1]);
        }
        cerr<<endl;
    }
    cerr<<endl;
}


// 获取锁类型
int pathlock_type(int x,int y) {
    return lockID[x][y];
}

// 获取锁类型
int pathlock_type(const coordinate2 &pos) {
    return pathlock_type(pos.x, pos.y);
}

// 判断目标时间窗能否被加锁
bool pathlock_isLockable(int rtidx, const pathlock_node &rhs, int id) {
    if (!id) return true;
    int n = lockSize[id] - 1;
    
    int flag = 1;
    // 枚举元素，判断是否有交集

    auto & lockStatus_ts = lockStatus_s[id];
    auto & lockStatus_te = lockStatus_e[id];

    for (int i = 0; i < ROBOT_SIZE; i++) {
        if (i == rtidx) continue;
        int s0 = lockStatus_ts[i].rank(rhs);
        int s1 = lockStatus_ts[i].rank(rhs.e_time + 1);
        int s2 = lockStatus_te[i].rank(rhs);
        if (s0 != s1 || s0 != s2) {
            if (--n < 0) {
                flag = 0;
                break;
            }
        }
    }
    
    return flag;
}

// 释放锁,type = 0 顺序释放锁，type = 1 倒序释放锁
void pathlock_release(int rtidx, int x, int y, int flag) {
    int id = lockID[x][y];

    if (!id) return;

    auto & lockStatus_ts = lockStatus_s[id][rtidx];
    auto & lockStatus_te = lockStatus_e[id][rtidx];
    auto lock = flag?lockStatus_ts.pop_back():lockStatus_ts.pop_front();

    if (lock.s_time <= 15000) {
        lockStatus_te.del(lock.e_time);
    }
}

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y, const pathlock_node& rhs) {
    int id = lockID[x][y], flag = 0;
    
    if (!id) return true;

    if (pathlock_isLockable(rtidx, rhs, id)) {
        lockStatus_s[id][rtidx].insert(rhs);
        lockStatus_e[id][rtidx].insert(rhs.e_time);
        flag = 1;
    }
    return flag;
}

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y, int s_time, int e_time) {
    return pathlock_acquire(rtidx, x, y, pathlock_node(s_time, e_time));
}


// 获取锁
bool pathlock_acquire(int rtidx, const coordinate2& pos, const pathlock_node& time) {
    return pathlock_acquire(rtidx, pos.x, pos.y, time);
}

// 判断节点在指定时间段是否可经过
bool pathlock_isReachable(int rtidx, int x, int y, int s_time, int e_time) {
    int id = lockID[x][y];
    return pathlock_isLockable(rtidx, pathlock_node(s_time, e_time), id);
}

// 判断节点在指定时间段是否可经过
bool pathlock_isReachable(int rtidx, int x, int y, const pathlock_node& time) {
    int id = lockID[x][y];
    return pathlock_isLockable(rtidx, time, id);
}

// 释放锁
void pathlock_release(int rtidx, const coordinate2 &pos, int flag) {
    pathlock_release(rtidx, pos.x, pos.y, flag);
}


// dijkstra调用时预估的时间
pathlock_node pathlock_getExpectTime(double dd){
    int r = dd/3 * 50  - 20;
    return pathlock_node(r, r + 150);
}

// 上锁时预估的时间,size用于表示剩余加入栈的节点数量
pathlock_node pathlock_getExpectTime(double dd , int size) {
    int r = frameID + max<int>(dd/3 * 50  - 320,0);
    int offest = (15000 - frameID)/size/size;
    return pathlock_node(1, 15000);
}