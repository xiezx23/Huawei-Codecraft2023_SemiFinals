#include "inc_io.hpp"


void readPlat() {
    char line[1024];
    for (int i = 0; i < 100; ++i) {
        fgets(plat[i], sizeof(line), stdin);
    }
    fgets(line, sizeof line, stdin); // receive OK
}

void readInfo() {
    char line[3];
    double x, y;
    scanf("%d %d",&curMoney, &K);
    for (int i = 0; i < K; ++i) {
        scanf("%d %lf %lf %d %d %d", 
            &wb[i].type,
            &x, &y, // location
            &wb[i].rtime,
            &wb[i].rstatus,
            &wb[i].pstatus
        ); wb[i].location.set(x, y);
    }
    for (int i = 0; i < ROBOT_SIZE; ++i) {
        rt[i].pcvc = rt[i].cvc;
        scanf("%d %d %lf %lf %lf %lf %lf",
            &rt[i].wb_id,
            &rt[i].pd_id,
            &rt[i].tvc,
            &rt[i].cvc,
            &rt[i].asp,
            &x, &y // lsp
        ); rt[i].lsp.set(x, y);
        scanf("%lf %lf %lf",
            &rt[i].toward,
            &x, &y // location
        ); rt[i].location.set(x, y);
    }
    getchar();
    fgets(line, sizeof line, stdin); // receive OK

}

void printRobotCommand(int robotId) {
    if (rt[robotId].cmd.sell)  printf("sell %d\n", robotId);
    if (rt[robotId].cmd.buy)  printf("buy %d\n", robotId);
    if (rt[robotId].cmd.destroy)  printf("destroy %d\n", robotId);
    printf("forward %d %f\n", robotId, rt[robotId].cmd.forward);
    printf("rotate %d %f\n", robotId, rt[robotId].cmd.rotate);
}

void debug(){
    for(int robotId = 0; robotId < 4; robotId++){
        if (rt[robotId].cmd.sell)  fprintf(stderr,"sell %d\n", robotId);
        if (rt[robotId].cmd.buy)  fprintf(stderr,"buy %d\n", robotId);
        if (rt[robotId].cmd.destroy)  fprintf(stderr,"destroy %d\n", robotId);
        fprintf(stderr,"forward %d %f\n", robotId, rt[robotId].cmd.forward);
        fprintf(stderr,"rotate %d %f\n", robotId, rt[robotId].cmd.rotate);
        fprintf(stderr,"curtask.buy %d\n", (int) rt[robotId].curTask.buy);
        fprintf(stderr,"curtask.sell %d\n",  (int)rt[robotId].curTask.sell);
        fprintf(stderr,"destId %d\n", rt[robotId].curTask.destId);
        fprintf(stderr,"nodeId %d\n\n\n", rt[robotId].nodeId);

    }
    cerr<<"cnt : "<<curFlow.cnt<<endl;
    cerr<<"curpool : "<<curFlow.curSize<<endl;
    cerr<<"curframeID : "<<frameID<<endl;
    cerr<<"flow : "<<curFlow.flow<<endl;
    cerr<<"--------------------------\n";

}
