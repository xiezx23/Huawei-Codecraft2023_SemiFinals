#include "inc_codecraft2023.hpp"
/*** 
 * @Description: 
 *      带势能场的 DWA 动态避障算法
 ***/
double dv0Max = 0.392155 * 50;       // 不携带物品时线最大加速度
double dv1Max = 0.280491 * 50;       // 携带物品时线最大加速度
double da0Max = 0.774646 * 50;       // 不携带物品时角最大加速度
double da1Max = 0.401561 * 50;       // 携带物品时角最大加速度

#define dwaN  1                  // 预测N帧
#define dwaM 10                  // 速度空间采样点数
const double dt = 0.02;         // 帧长度

double dwa_para1 = -1;         // 势能分量系数
double dwa_para2 = -9;        // 目标角度系数
double dwa_para3 = 1;        // 有效速度系数


double recordData[4][dwaM][dwaM];
double recordSum[4];

void cleanData() {
    for (int i = 0; i < 4; ++i) {
        recordSum[i] = 0;
        for (int j = 0; j < dwaM; ++j) {
            memset(recordData[i][j], 0, sizeof(recordData[i][j]));
        }
    }
}

double getEvaluate(int vidx, int aidx) {
    if (recordData[3][vidx][aidx] == INF) return -INF;
    double pe = recordData[0][vidx][aidx] / recordSum[0];
    double heading = recordData[1][vidx][aidx] / recordSum[1];
    double v = recordData[2][vidx][aidx] / recordSum[2];
    return  dwa_para1*pe + dwa_para2*heading + dwa_para3*v;
}

// G(v, w) = dwa_para1*Pe(position) + dwa_para2*H(loca, dest, speed) + dwa_para3*V(speed)
void motionEvaluate(coordinate position,int rtIdx,vec speed, int vidx, int aidx) {
    // Detect the location to confirm it is in the map
    robot& rbt = rt[rtIdx];
    coordinate& dest = rbt.haveTemDest ? rbt.temDest : rbt.curTask.destCo;
    double robotRadius = 0.45;  // the radius of robot
    if (rbt.pd_id) {
        robotRadius = 0.53;     // the radius when carrying products
    }
    robotRadius += 0.1;

    // check distence from robot to the wall
    if (fabs(position.x) <= robotRadius || fabs(position.x - 50.0) <= robotRadius) {
        recordData[3][vidx][aidx] = INF;
        return;
    }
    else if (fabs(position.y) <= robotRadius || fabs(position.y - 50.0) <= robotRadius) {
        recordData[3][vidx][aidx] = INF;
        return;
    }

    // caculate Pe(position)
    double pe = 0;
    for (int otherRtIdx = 0; otherRtIdx < ROBOT_SIZE; ++otherRtIdx) {
        if (otherRtIdx != rtIdx) {
            pe += cntPontEnergy(otherRtIdx, position);
        }
    }
    if (pe < 0.3) pe = 0;
    recordData[0][vidx][aidx] = pe;
    recordSum[0] += pe;

    // caculate H(position, dest, speed)
    vec p2d(dest.x - position.x, dest.y - position.y);
    double heading = cntAngle(p2d, speed);
    recordData[1][vidx][aidx] = heading;
    recordSum[1] += heading;

    // caculate V(speed), velocity to destination
    double v = dotProduct(p2d, speed) / modulusOfvector(p2d);
    recordData[2][vidx][aidx] = v;
    recordSum[2] += v;
}

vec motionPredict(int rtIdx) {
    const double eps = 1e-6;

    // 获取机器人最大加速度
    robot &bot = rt[rtIdx];
    int type = bot.pd_id > 0;
    double dvMax = type ? dv1Max : dv0Max;
    double daMax = type ? da1Max : da0Max;


    // 设置速度单位采样变化量
    dvMax = dvMax * dt;
    vec &curSpeed = bot.lsp;
    double curLineSpeed = modulusOfvector(curSpeed);
    double dv = dvMax / dwaM;

    // 设置角速度单位采样变化量
    daMax = daMax * dt;
    double &curAsp = bot.asp, da = daMax / dwaM;

    // 维护当前位置和朝向
    vec &curLocation = bot.location;
    double ly = curLocation.y, lx = curLocation.x, tmpy, tmpx;
    double curToward = bot.toward;

    double tmpasp = curAsp, tmpLineSpeed = curLineSpeed;

    // 维护最佳路径
    double score = -INF;
    vec best = vec(curLineSpeed,curAsp);

    // 根据设置的1帧后的速度和角速度，预测N帧后的位置和朝向，并维护最佳路径评估得分
    auto pathEvaluate = [&]() {
        double w1 = (tmpasp + curAsp) / 2.0,v1 = (tmpLineSpeed + curLineSpeed) / 2.0;
        tmpy = ly, tmpx = lx;

        // 假设速度大小、角速度变化均匀，故第一帧按平均速度、角速度计算
        double tmpToward = curToward + dt * w1;
        if (fabs(w1) <= eps) {
            tmpx += v1 * dt * cos(curToward);
            tmpy += v1 * dt * sin(curToward);
        } else {
            tmpx += v1/w1 * (sin(curToward) - sin(tmpToward));
            tmpy -= v1/w1 * (cos(curToward) - cos(tmpToward));
        }

        // 假设后N-1帧速度大小、角速度不改变，计算位置和朝向
        w1 = tmpasp, v1 = tmpLineSpeed;
        if (fabs(w1) <= eps) {
            double tmp = v1 * dt * (dwaN - 1);
            tmpx += tmp * cos(tmpToward);
            tmpy += tmp * sin(tmpToward);
        } else {
            double tmp = w1 * dt * (dwaN - 1);
            tmpx += v1/w1 * (sin(tmpToward) - sin(tmpToward + tmp));
            tmpy -= v1/w1 * (cos(tmpToward) - cos(tmpToward + tmp));
            tmpToward += tmp;
        }

        double tmpScore = motionEvaluate(vec(tmpx,tmpy),rtIdx,vec(v1*cos(tmpToward),v1*sin(tmpToward)));
        if (tmpScore > score) score = tmpScore, best.set(tmpLineSpeed,tmpasp);
    };

    // 总采样点数 : 4M^2 + 1
    int left, i = 0, offest = 0;
    double leftasp;

    // 设置左侧采样空间偏移量,保留[-dwaM]
    i = (curLineSpeed - offest) / dv;
    i = max(-i - 2 , -dwaM + 1);

    left = (curAsp + PI) / da;
    left = max(-left - 2, -dwaM);

    leftasp = curAsp + left * da;
    while (leftasp <= -PI - eps) leftasp += da, ++left;

    {
        tmpLineSpeed = max(-2.0,min(6.0,tmpLineSpeed));
        tmpasp = min(leftasp,PI);
        for (int j = left; j <= dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            pathEvaluate();
        }
    }

    tmpLineSpeed = curLineSpeed + i * dv;
    while (tmpLineSpeed <= offest - eps) tmpLineSpeed += dv, ++i;



    for (; i <= dwaM; i++,tmpLineSpeed += dv) {
        if (tmpLineSpeed > 6 + eps) break;
        tmpasp = leftasp;
        for (int j = left; j <= dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            pathEvaluate();
        }
    }


    return best;
}