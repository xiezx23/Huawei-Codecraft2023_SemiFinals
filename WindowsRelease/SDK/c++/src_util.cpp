#include "inc_util.hpp"

void logInfo::addWallCol(int rtIdx) {
    ++wallCollisionNum[rtIdx];
}

void logInfo::addRoboCol(int rtIdx) {
    ++roboCollisionNum[rtIdx];
}

void logInfo::buyProduct(int rtIdx, int pdIdx) {
    ++buyNum[rtIdx][pdIdx];
}

void logInfo::sellProduct(int rtIdx, int pdIdx) {
    ++sellNum[rtIdx][pdIdx];
    ++totalSellNum[pdIdx];
}

// 输出地图的日志信息
void logInfo::printLog() {
    std::ofstream fout("log.txt", std::ios_base::app); 
    fout << "******************************LOG INFORMATION START******************************";
    fout << endl << setw(15) << "ROBOT:";
    for (int i = 1; i <= ROBOT_SIZE; ++i)    fout << setw(8) << i;
    fout << setw(8) << "TOTAL";

    int sum = 0;
    fout << endl << setw(15) << "WALL COLLOSION:";
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        fout << setw(8) << wallCollisionNum[i];
        sum += wallCollisionNum[i];
    }   
    fout << setw(8) << sum;
    sum = 0;
    fout << endl << setw(15) << "ROBO COLLOSION:";
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        fout << setw(8) << roboCollisionNum[i];
        sum += roboCollisionNum[i];
    }   
    fout << setw(8) << sum;
    
    for (int i = 1; i <= 7; ++i) {
        fout << endl << setw(10) << i << "_BUY:";
        sum = 0;
        for (int j = 0; j < ROBOT_SIZE; ++j) {
            fout << setw(8) << buyNum[j][i];
            sum += buyNum[j][i];
        }
        fout << setw(8) << sum;
        fout << endl << setw(9) << i << "_SELL:";
        sum = 0;
        for (int j = 0; j < ROBOT_SIZE; ++j) {
            fout << setw(8) << sellNum[j][i];
            sum += sellNum[j][i];
        }
        fout << setw(8) << sum;
    }
    fout << endl << "******************************LOG INFORMATION END*******************************" << endl << endl << endl;
    fout.close();
}