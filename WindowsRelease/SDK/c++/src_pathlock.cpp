#include "inc_pathlock.hpp"

char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
int pathdetect_f[(MAP_SIZE + 2)*(MAP_SIZE + 2)];

int lockID[MAP_SIZE][MAP_SIZE];
int lockCnt = 1;
int lockStatus[MAP_SIZE * MAP_SIZE];
int robotStatus[4][MAP_SIZE * MAP_SIZE];

std::mutex path_mutex; 

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
    for (int i = 0; i < MAP_SIZE; i++) {
        resolve_plat[i][0] = resolve_plat[i][MAP_SIZE + 1] = '#';
        resolve_plat[0][i] = resolve_plat[MAP_SIZE + 1][i] = '#';
    }

    for (int i = 0; i <= MAP_SIZE + 1; i++) 
            for (int j = 0; j <= MAP_SIZE + 1; j++)  
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
        for (int i = 0; i <= MAP_SIZE + 1; i++) {
            for (int j = 0; j <= MAP_SIZE + 1; j++) {
                if (resolve_plat[i][j] == '#') {
                    for (int di = 0; di <= dM; di++) {
                        if (i + di > MAP_SIZE + 1) break;
                        for (int dj = -dM; dj <= dM; dj++) {
                            if (j + dj > MAP_SIZE + 1) break;
                            if (j + dj < 0) break;
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

    // 将所有中心位置与墙壁的距离不大于r的离散坐标标记为c
    //auto rLabel2 = [](double r, char c){
    //    double boundary = ((r + 0.25) * 2);
    //    const int dM = ceil(boundary);   
    //    boundary = boundary * boundary;     
    //    for (int i = 0; i <= MAP_SIZE + 1; ++i) {
    //        for (int j = 0; j <= MAP_SIZE + 1; ++j) {
    //            if (resolve_plat[i][j] == '#') {
    //                for (int di = -dM; di <= dM; ++di) {
    //                    if (i+di < 1 || i+di > MAP_SIZE) continue;
    //                    for (int dj = -dM; dj <= dM; ++dj) {
    //                        if (j+dj < 1 || j+dj > MAP_SIZE) continue;
    //                        if (resolve_plat[i+di][j+dj] == '.') {
    //                            if (di * di + dj * dj <= boundary) {
    //                                resolve_plat[i+di][j+dj] = c;
    //                            }
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //    }
    //};

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
    lockStatus[0] = 0x3f3f3f;
    for (int i = 1; i < lockCnt; i++) lockStatus[i] = 1;
    #ifdef PATH_DEBUG
    fprintf(stderr,"lockCnt : %d\n", lockCnt);
    #endif

    // 仅允许不携带物品的机器人通过
    // rLabel2(0.53, '3');
}

// 输出处理后的地图
void printMap() {
    char ma[256];
    for (int i = 0; i < 26; i++) ma[i] = 'a' + i;
    for (int i = 26; i < 52; i++) ma[i] = 'A' + i - 26;

    // 输出处理后的地图
    for (int j = MAP_SIZE - 1; j + 1; --j) {
        for (int i = 0; i < MAP_SIZE; i++) {
            fprintf(stderr,"%c", resolve_plat[i+1][j+1]);
            // if (lockID[i][j]) fprintf(stderr,"%c", ma[lockID[i][j]%52]);
            // else fprintf(stderr,"%c", resolve_plat[i+1][j+1]);
        }
        cerr<<endl;
    }
    cerr<<endl;
}

// 释放锁
void pathlock_release(int rtidx, int x, int y) {
    int id = lockID[x][y];
    #ifdef PATH_DEBUG
    fprintf(stderr,"release curframeID : %d lockID : %d,robot : %d lockStatus : %d robotStatus : %d\n", frameID, id, rtidx, lockStatus[id], robotStatus[rtidx][id]);
    #endif

    if (robotStatus[rtidx][id]) if(!--robotStatus[rtidx][id]) ++lockStatus[id];
}

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y) {
    int id = lockID[x][y], flag = 0;
    #ifdef PATH_DEBUG
    fprintf(stderr,"acquire curframeID : %d lockID : %d,robot : %d lockStatus : %d robotStatus : %d\n", frameID, id, rtidx, lockStatus[id], robotStatus[rtidx][id]);
    #endif
    
    if (robotStatus[rtidx][id] > 0) ++robotStatus[rtidx][id],flag = 1;
    else if (lockStatus[id] > 0) --lockStatus[id], ++robotStatus[rtidx][id], flag = 1;
    
    #ifdef PATH_DEBUG
    fprintf(stderr,"acquire %s curframeID : %d lockID : %d,robot : %d\n", flag?"success":"fail", frameID, id, rtidx);
    #endif
    return flag;
}

// 判断节点是否可经过
bool pathlock_isReachable(int rtidx, int x, int y) {
    int id = lockID[x][y];
    return (robotStatus[rtidx][id] > 0 || lockStatus[id] > 0);
}

// 获取锁类型
int pathlock_type(int x,int y) {
    return lockID[x][y];
}

// 释放锁
void pathlock_release(int rtidx, const coordinate2 &pos) {
    return pathlock_release(rtidx, pos.x, pos.y);
}

// 获取锁
bool pathlock_acquire(int rtidx, const coordinate2 &pos) {
    return pathlock_acquire(rtidx, pos.x, pos.y);
}

// 判断节点是否可经过
bool pathlock_isReachable(int rtidx, const coordinate2 &pos) {
    return pathlock_isReachable(rtidx, pos.x, pos.y);
}

// 获取锁类型
int pathlock_type(const coordinate2 &pos) {
    return pathlock_type(pos.x, pos.y);
}

// 查询指定点的上锁情况
void pathlock_getStatus(const coordinate2 &pos) {
    pathlock_getStatus(pos.x, pos.y);
}

// 查询指定点的上锁情况
void pathlock_getStatus(int x, int y) {
    fprintf(stderr,"lock_Status frameId : %d lock_Id : %d pos : (%d,%d) lockStatus : %d\n", frameID,lockID[x][y], x, y, lockStatus[lockID[x][y]]);
    for(int i = 0; i < ROBOT_SIZE; i++) {
        coordinate2 pos = rt[i].location;
        fprintf(stderr,"\trobot %d : %d %d %d\n", i, robotStatus[i][lockID[x][y]],pos.x, pos.y);
    }
    fprintf(stderr,"\n");
}