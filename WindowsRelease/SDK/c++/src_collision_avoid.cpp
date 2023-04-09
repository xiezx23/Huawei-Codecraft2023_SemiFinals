/*** 
 * @Description: 
 *      引入势能场的概念，把碰撞处理从紧急避让变成引入势能较低点作为临时目的地（3-17）
 *      对势能场分布进行修正，加入角度考量（3-19）
 ***/
#include "inc_codecraft2023.hpp"

// 计算势能分布 a 为角度，lsp 为线速度向量
double cntR(double a, vec& lsp, double asp) {
    double lm = modulusOfvector(lsp);
    // a -= 0.0002 * asp;         // 角度偏置
    double e_up = -1 * a * a * lm * lm / 36;
    return exp(e_up) * lm / 6;
}

// 计算机器人 rtIdx 对点 d 的势能
double cntPontEnergy(int rtIdx, coordinate& d) {
    robot& rbt = rt[rtIdx];
    vec r2d(d.x - rbt.location.x, d.y - rbt.location.y); 
    double angle = cntAngle(rbt.lsp, r2d);
    // 根据相对机器人左正右负设定角度符号
    if (crossProduct(rbt.lsp, r2d) > 0) {
        angle = -angle;
    }
    return 1.2 * cntR(angle, rbt.lsp, rbt.asp) / dis(d, rbt.location);
}

void collitionAvoidance() {
    double u = 0.3; // 拥塞阈值
    for (int curRt = 0; curRt < ROBOT_SIZE; ++curRt) {
        // if (rt[curRt].haveTemDest) continue;
        // 枚举每个机器人，计算其碰撞势能检测点受到的势能
        double pe = 0.0;                        // potentail energy
        pair<double, int> maxPeComponent(0,0);  // 维护势能分量最大的机器人
        coordinate detectPoint;                 // 探测点
        coordinate& rLoca = rt[curRt].location;
        vec& lsp = rt[curRt].lsp;
        detectPoint.set(rLoca.x + 0.4 * lsp.x, rLoca.y + 0.4 * lsp.y);
        for (int otherRt = 0; otherRt < ROBOT_SIZE; ++otherRt) {
            if (curRt == otherRt) continue;
            double peComponent = cntPontEnergy(otherRt, detectPoint);
            pe += peComponent;
            if (maxPeComponent.first < peComponent) {
                // 记录最大势能分量贡献者
                maxPeComponent.first = peComponent;
                maxPeComponent.second = otherRt;
            }
        }
        if (pe >= u) {
            // 需要进行碰撞避免，进行让路者选举
            double lm1 = modulusOfvector(lsp);
            double lm2 = modulusOfvector(rt[maxPeComponent.second].lsp);
            // if (lm2 <= 1 && lm1 > lm2) {
            //     // 当本方速度高于对方且对方速度小于1.2时，本方避让
            //     double dis_para = 0.2;
            //     coordinate& otLoca = rt[maxPeComponent.second].location;
            //     vec otVec; otVec.set(rLoca.x-otLoca.x, rLoca.y-otLoca.y);
            //     vec l_lsp(otVec.y, -otVec.x); 
            //     vec r_lsp(-otVec.y, otVec.x); 
            //     coordinate aLeft(otLoca.x + dis_para * l_lsp.x, otLoca.y + dis_para * l_lsp.y);
            //     coordinate aRight(otLoca.x + dis_para * r_lsp.x, otLoca.y + dis_para * r_lsp.y);
            //     double aLeftPe = 0, aRightPe = 0;
            //     for (int otherRt = 0; otherRt < ROBOT_SIZE; ++otherRt) {
            //         if (curRt == otherRt) continue;
            //         aLeftPe += cntPontEnergy(otherRt, aLeft);
            //         aRightPe += cntPontEnergy(otherRt, aRight);
            //     }
            //     if (aLeftPe <= aRightPe) {
            //         rt[curRt].setTemporaryDest(aLeft);
            //     }
            //     else {
            //         rt[curRt].setTemporaryDest(aRight);
            //     }
            // }
            if (lm1 <= lm2 || (K == 18 && rt[curRt].pd_id == 0)) {
                // fprintf(stderr, "cur speed:%.2f\n",lm1);
                // 两个避让候选点根据势能选择低势能者为临时目的地
                double rot = PI/6;
                double dis_para = 0.2;
                vec l_lsp(lsp.x * cos(rot) + lsp.y * sin(rot), -lsp.x * sin(rot) + lsp.y * cos(rot));     // 逆时针旋转60°
                vec r_lsp(lsp.x * cos(-rot) + lsp.y * sin(-rot), -lsp.x * sin(-rot) + lsp.y * cos(-rot)); // 顺时针旋转60°
                coordinate aLeft(rLoca.x + dis_para * l_lsp.x, rLoca.y + dis_para * l_lsp.y);
                coordinate aRight(rLoca.x + dis_para * r_lsp.x, rLoca.y + dis_para * r_lsp.y);
                double aLeftPe = 0, aRightPe = 0;
                for (int otherRt = 0; otherRt < ROBOT_SIZE; ++otherRt) {
                    if (curRt == otherRt) continue;
                    aLeftPe += cntPontEnergy(otherRt, aLeft);
                    aRightPe += cntPontEnergy(otherRt, aRight);
                }
                if (aLeftPe <= aRightPe) {
                    rt[curRt].setTemporaryDest(aLeft);
                }
                else {
                    rt[curRt].setTemporaryDest(aRight);
                }
                // fprintf(stderr,"rt[%d] local:%.2f %.2f  tmp:%.2f %.2f\n",rt[curRt].rtIdx,rLoca.x, rLoca.y, rt[curRt].temDest.x, rt[curRt].temDest.y);
            }
        }
    }
    return;
}

void ori_collitionAvoidance() {
    // 检测机器人之间的运动向量，估计碰撞可能
    double colDiss = 3;
    for (int rt1 = 0; rt1 < ROBOT_SIZE; ++rt1) {
        
        // if (rt[rt1].holdTime == 0)
        
        for (int rt2 = 0; rt2 < ROBOT_SIZE; ++rt2) {
            if (rt1 == rt2) continue;
            coordinate& a = rt[rt1].location;
            coordinate& b = rt[rt2].location;
            if (dis(a, b) < colDiss) {
                vec& lsp = rt[rt1].lsp;
                vec disVec; 
                disVec.set(b.x - a.x, b.y - a.y);
                if (dotProduct(lsp, disVec) > 0) { // 两者夹角小于90度
                    double ratio  = 10.0 / (1 + 9 * dis(a, b));
                    double cp = crossProduct(lsp, disVec);
                    double ag = dotProduct(rt[rt1].lsp, rt[rt2].lsp);
                    double para1 = min(PI, ratio * 2.5 * 1.8);
                    double para2 = min(6.0, 6.0 / ratio);
                    if (ag < 0){
                        if (cp < 0) {
                            // 对方在右手边，逆时针旋转
                            rt[rt1].avoidance.set(-5,para1);
                        } else {
                            // 左手边，顺时针旋转
                            rt[rt1].avoidance.set(-5,-para1);
                        }
                    }else{
                        para2 = min(rt[rt1].cmd.forward, para2);
                        rt[rt1].avoidance.set(5, para2);

                    }
                    
                    rt[rt1].holdTime = 5; // 持续时间帧数
                    // cerr << "collision avoidance \n";
                }
            }
        }
        if (rt[rt1].holdTime > 0) {
            if (rt[rt1].holdTime != 5) {
                rt[rt1].holdTime = rt[rt1].holdTime / 2;
            }
            if(rt[rt1].avoidance.x < 0){
                rt[rt1].cmd.rotate = rt[rt1].avoidance.y;
            }else{
                rt[rt1].cmd.forward = rt[rt1].avoidance.y;
            }
        }
    }
}
