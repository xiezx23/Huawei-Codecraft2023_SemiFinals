/*** 
 * @Author: Xzh
 * @Date: 2023-03-12 22:50:56
 * @LastEditTime: 2023-03-21 15:15:
 * @LastEditors: Zym
 * @Description: 
 *      1、当朝向与目的地偏差大时，以最大角速度调整朝向，当小于一个阈值时，对朝向进行微调
 *      2、线速度由朝向与目的地的夹角确定，夹角越大，线速度越小；当朝向为目的方向时，以最大线速度前进（避免行星运动）
 *      3、当机器人与墙的距离小于某个阈值时，检测是否会撞墙，若会，降低线速度（避免撞墙）(未实现)
 ***/
#include "inc_codecraft2023.hpp"

void robot::setSpeed(coordinate dest) {
    // 参数
    double dx = dest.x - location.x;
    double dy = dest.y - location.y;
    double dist = sqrt(dx * dx + dy * dy);
    double angle = atan2(dy, dx);
    double angleDiff = angle - toward;
    double sign = 0;
    double absAngleDiff = 0;    
    // 将加夹角差转换至[-PI, PI]区间
    if (angleDiff > PI) angleDiff -= 2 * PI;
    if (angleDiff < -PI) angleDiff += 2 * PI;
    if (angleDiff > 0) sign = 1.0;
    else if (angleDiff < 0) sign = -1.0;
    absAngleDiff = sign*angleDiff;

    // Limit the angular velocity according to the angle
    // double rotatePara = 8; try until here
    double rotatePara = 15;
    if (absAngleDiff * rotatePara > PI) {
        cmd.rotate = sign * PI;
    }
    else {
        cmd.rotate = sign * rotatePara * absAngleDiff;
    }
    // Limit the velocity according to the angle
    if (absAngleDiff * 6 > PI) {
        cmd.forward = 2 * cos(absAngleDiff);
    }
    else {
        cmd.forward = 6 * cos(absAngleDiff); 
    }
    // Limit the velocity according to the distance to destination
    double l2d = dis(curTask.destCo, location);
    if (l2d < 2) cmd.forward /= 2;
    // // 对撞墙进行特判
    // coordinate detectPoint;                 // 探测点
    // double para3 = 0.1;
    // detectPoint.set(location.x + para3 * lsp.x, location.y + para3 * lsp.y);
    // if ((detectPoint.x <= 1 && lsp.x < 0) || (detectPoint.x >= 50 - 1 && lsp.x > 0)) {
    //     if (cmd.forward > 0) cmd.forward = 0.2 * cmd.forward;
    // }
    // else if ((detectPoint.y <= 1 && lsp.y < 0) || (detectPoint.y >= 50 - 1 && lsp.y > 0)) {
    //     if (cmd.forward > 0) cmd.forward = 0.2 * cmd.forward;
    // }

    // 当靠近墙边时，检测是否可能撞墙，若可能，则减速
    // double frame = 0.04;
    // double t1 = (sin(toward+cmd.rotate*frame)-sin(toward)) / cmd.rotate;
    // double t2 = -(cos(toward+cmd.rotate*frame)-cos(toward)) / cmd.rotate;
    // double vx = cmd.forward * cos(toward);
    // double vy = cmd.forward * sin(toward);    
    // double lowerbound = 0.65;
    // double upperbound = 50 - lowerbound;
    // double minLSpeed = 6.0;
    // // if ((location.x + cmd.forward * t1 <= lowerbound) && (vx < 0)) {
    // //     minLSpeed = (lowerbound-location.x) / t1;
    // // }
    // // else if ((location.x + cmd.forward * t1 >= upperbound) && (vx > 0)) {
    // //     minLSpeed = (upperbound-location.x) / t1;
    // // }
    // // cmd.forward = min(minLSpeed, cmd.forward);

    // minLSpeed = 6.0;
    // if ((location.y + frame * vy <= lowerbound) && (vy < 0)) {
    //     minLSpeed = (lowerbound-location.y) / t2;
    // }
    // else if ((location.y + frame * vy >= upperbound) && (vy > 0)) {
    //     minLSpeed = (upperbound-location.y) / frame;
    //     // minLSpeed = cmd.forward / abs(cmd.forward);
    // }
    // cmd.forward = min(minLSpeed, cmd.forward);
}