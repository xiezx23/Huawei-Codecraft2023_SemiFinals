#include "inc_codecraft2023.hpp"
/*** 
 * @Description: 
 *      带势能场的 DWA 动态避障算法
 ***/
double dv0Max = 0.392155 * 50;       // 不携带物品时线最大加速度
double dv1Max = 0.280491 * 50;       // 携带物品时线最大加速度
double da0Max = 0.774646 * 50;       // 不携带物品时角最大加速度
double da1Max = 0.401561 * 50;       // 携带物品时角最大加速度

#define dwaN  6                  // 预测N帧
#define dwaM 15                  // 速度空间采样点数
const double dt = 0.02;         // 帧长度

double dwa_para1 = -12.5;         // 势能分量系数
double dwa_para2 = 10;        // 目标角度系数
double dwa_para3 = 5;        // 有效速度系数


double recordData[2*dwaM][2*dwaM][4];
double recordSum[3];

void cleanData() {
    memset(recordSum, 0, sizeof(recordSum));
    memset(recordData, 0, sizeof(recordData));
}

double getEvaluate(int vidx, int aidx) {
    if (recordData[vidx][aidx][3] == INF) return -INF;
    double pe = recordData[vidx][aidx][0] / recordSum[0];
    if (isnan(pe)) pe = 0;
    double heading = 1 - recordData[vidx][aidx][1] / recordSum[1];
    if (isnan(heading)) heading = 0;
    double v = recordData[vidx][aidx][2] / recordSum[2];
    if (isnan(v)) v = 0;
    double ans = dwa_para1 * pe + dwa_para2 * heading + dwa_para3 * v;
        // cerr << pe << ' ' << heading << ' ' << v << endl;
    return ans;
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
    robotRadius += 1;

    // check distence from robot to the wall
    if (fabs(position.x) <= robotRadius || fabs(position.x - 50.0) <= robotRadius) {
        recordData[vidx][aidx][3] = INF;
        return;
    }
    else if (fabs(position.y) <= robotRadius || fabs(position.y - 50.0) <= robotRadius) {
        recordData[vidx][aidx][3] = INF;
        return;
    }

    // caculate Pe(position)
    double pe = 0;
    for (int otherRtIdx = 0; otherRtIdx < ROBOT_SIZE; ++otherRtIdx) {
        if (otherRtIdx != rtIdx) {
            pe += cntPontEnergy(otherRtIdx, position);
        }
    }
    recordData[vidx][aidx][0] = pe;
    recordSum[0] += pe;

    // caculate H(position, dest, speed)
    vec p2d(dest.x - position.x, dest.y - position.y);
    double heading = cntAngle(p2d, speed);
    recordData[vidx][aidx][1] = heading;
    recordSum[1] += heading;

    // caculate V(speed)
    vec l2p(rbt.location.x - position.x, rbt.location.y - position.y);
    double v = dotProduct(l2p, speed) / modulusOfvector(l2p);
    recordData[vidx][aidx][2] = v;
    recordSum[2] += fabs(v);
}

vec motionPredict(int rtIdx) {
    cleanData();
    
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
    double score = -INF*2;
    vec best = vec(curLineSpeed,curAsp);

    // 根据设置的1帧后的速度和角速度，预测N帧后的位置和朝向，并维护最佳路径评估得分
    auto pathEvaluate = [&](int i,int j) {
        double w1 = (tmpasp + curAsp) / 2.0,v1 = (tmpLineSpeed + curLineSpeed) / 2.0;
        double dda = tmpasp - curAsp, ddv = tmpLineSpeed - curLineSpeed;
        double tmpToward = curToward;

        tmpy = ly, tmpx = lx;

        // 假设速度大小、角速度变化均匀，故第一帧按平均速度、角速度计算
        for(int k = 0; k < dwaN; k++) {
            double nextToward = tmpToward + w1 * dt;
            if (fabs(w1) <= eps) {
                tmpx += v1 * dt * cos(tmpToward);
                tmpy += v1 * dt * sin(tmpToward);
            } else {
                tmpx += v1/w1 * (sin(tmpToward) - sin(nextToward));
                tmpy -= v1/w1 * (cos(tmpToward) - cos(nextToward));
            }
            w1 += dda, v1 += ddv, tmpToward = nextToward;
        }

        motionEvaluate(vec(tmpx,tmpy),rtIdx,vec(v1*cos(tmpToward),v1*sin(tmpToward)),i,j);
    };

    // 总采样点数 : 4M^2 + 1
    int left, i = 0, offest = 0, lefti;
    double leftasp;

    // 设置左侧采样空间偏移量,保留[-dwaM]
    i = (curLineSpeed - offest) / dv;
    i = max(-i - 2 , -dwaM + 1);

    left = (curAsp + PI) / da;
    left = max(-left - 2, -dwaM + 1);

    leftasp = curAsp + left * da;
    while (leftasp <= -PI - eps) leftasp += da, ++left;

    {
        tmpLineSpeed = max(-2.0,min(6.0,curLineSpeed));
        tmpasp = min(curAsp,PI);
        for (int j = left; j < dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            pathEvaluate(0,j + dwaM);
        }
    }

    tmpLineSpeed = curLineSpeed + i * dv;
    while (tmpLineSpeed <= offest - eps) tmpLineSpeed += dv, ++i;
    lefti = i;

    for (; i < dwaM; i++,tmpLineSpeed += dv) {
        if (tmpLineSpeed > 6 + eps) break;
        tmpasp = leftasp;
        for (int j = left; j < dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            pathEvaluate(i + dwaM, j + dwaM);
        }
    }

    auto checkValue = [&](int i, int j) {
        double value = getEvaluate(i,j);
        // if (rtIdx == 1) fprintf(stderr,"frameId: %d, value: %f\n",frameID, value);
        if (value > score) score = value, best.set(tmpLineSpeed,tmpasp);
    };

    {
        tmpLineSpeed = max(-2.0,min(6.0,curLineSpeed));
        tmpasp = min(curAsp,PI);
        for (int j = left; j < dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            checkValue(0,j + dwaM);
        }
    }

    i = lefti, tmpLineSpeed = curLineSpeed + i * dv;

    for (; i < dwaM; i++,tmpLineSpeed += dv) {
        if (tmpLineSpeed > 6 + eps) break;
        tmpasp = leftasp;
        for (int j = left; j < dwaM; j++, tmpasp += da) {
            if (tmpasp > PI + eps) break;
            checkValue(i+dwaM,j+dwaM);
        }
    }

    return best;
}