#include "inc_pathdetect.hpp"

char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
int pathdetect_f[(MAP_SIZE + 2)*(MAP_SIZE + 2)];

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



void pathdect_init() {
    // 转换地图
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            resolve_plat[i + 1][j + 1] = isdigit(plat[i][j])?'.':plat[i][j];
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
                                if (sqrt(di * di + dj * dj) <= 4 * r) {
                                    if (!di) {
                                        for (int k = j + 1; k < j + dj; k++) {
                                            if (resolve_plat[i][k] == '.') {
                                                resolve_plat[i][k] = c;
                                            }
                                        }
                                    } else if(!dj) {
                                        for (int k = i + 1; k < i + di; k++) {
                                            if (resolve_plat[k][j] == '.') {
                                                resolve_plat[k][j] = c;
                                            }
                                        }
                                    } else if(di == 2 && abs(dj) == 2) {
                                        resolve_plat[i + 1][j + dj/2] = c;
                                    } 
                                    // else if(di + abs(dj) == 5) {
                                    //     int sgnj = dj/abs(dj);
                                    //     if (di == 2) {
                                    //         resolve_plat[i + 1][j + sgnj] = c;
                                    //         resolve_plat[i + 1][j + sgnj * 2] = c;
                                    //     } else {
                                    //         resolve_plat[i + 1][j + sgnj] = c;
                                    //         resolve_plat[i + 2][j + sgnj] = c;
                                    //     }
                                    // }
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    rLabel(0.99,'2');
    rLabel(0.48,'1');

    // 合并字符为'2'的连通域，并用a-z标记
    char now = 'a';
    for (int i = 0; i <= MAP_SIZE + 1; i++) {
        for (int j = 0; j <= MAP_SIZE + 1; j++) {
            if (resolve_plat[i][j] == '2') {
                int pos = i * (MAP_SIZE + 2) + j;
                int fa = pathdetect_find(pos);
                if (fa == pos) {
                    resolve_plat[i][j] = now++;
                    if(now > 'z') now = 'B';
                } else {
                    resolve_plat[i][j] = resolve_plat[fa / (MAP_SIZE + 2)][fa % (MAP_SIZE + 2)];
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

    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            cerr<<resolve_plat[i + 1][j + 1];
        }
        cerr<<endl;
    }
    cout<<1;


}