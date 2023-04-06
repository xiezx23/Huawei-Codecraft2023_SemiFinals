#include "inc_route_correction.hpp"

pair<int, int> changePoint[] = {
    make_pair(1,0), make_pair(-1,0), make_pair(0,1), make_pair(0,-1),
    make_pair(1,1), make_pair(1,-1), make_pair(-1,1), make_pair(-1,-1)
};
coordinate pointCorrection(coordinate2&& oriPosi) {
    auto checkInLegal = [](int x, int y) {
        if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) return false;
        return true;
    };
    for (int i = 0; i < 8; ++i) {
        pair<int,int>& cp = changePoint[i];
        int check_x = oriPosi.x + cp.first;
        int check_y = oriPosi.y + cp.second;
        if (checkInLegal(check_x, check_y)) {
            if (plat[check_x][check_y] == '#') {
                // cerr << "Here is a wall:" << check_x << ' ' << check_y << endl;
                int rela_x = oriPosi.x - cp.first;
                int rela_y = oriPosi.y - cp.second;
                if (checkInLegal(rela_x,rela_y) && plat[rela_x][rela_y] == '.') {
                    // 迁移坐标
                    // cerr << "Here is a .:" << rela_x << ' ' << rela_y << endl;
                    double cur_x = oriPosi.x + rela_x;
                    double cur_y = oriPosi.y + rela_y;
                    coordinate posi(cur_x/4 + 0.25, cur_y/4 + 0.25);
                    return posi;
                }
            }
        }
    }
    return oriPosi;
}