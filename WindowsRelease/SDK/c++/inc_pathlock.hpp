#ifndef PATHLOCK_HPP
#define PATHLOCK_HPP
#include "inc_codecraft2023.hpp"


// 维护基于时间窗的锁
struct pathlock_node{
    int s_time, e_time;
    pathlock_node(){}
    pathlock_node(int s_time, int e_time = 15000) : s_time(s_time), e_time(e_time) {}
    pathlock_node(const coordinate2 &c) : s_time(c.x), e_time(c.y) {}
    bool operator < (const pathlock_node &rhs) const {
        return s_time < rhs.s_time;
    }
    void set(int s_time, int e_time) {
        this->s_time = s_time;
        this->e_time = e_time;
    }
    operator coordinate2() const{
        return coordinate2(s_time, e_time);
    }
    operator int() const{
        return s_time;
    }
};

template <typename T>
struct fhq_treap{
    //rd为优先级，越小越高
    int rt;
    std::vector<int> rd, sz, son[2], leftsize;
    std::vector<T> val;

    inline int rand(){
        static int seed = 2333;
        return seed = (int)((((seed ^ 998244353) + 19260817ll) * 19890604ll) % 1000000007);
    }
    int get_node() {
        int u;
        if (leftsize.empty()) {
            u = rd.size();
            rd.push_back(rand());
            sz.push_back(1);
            son[0].push_back(0);
            son[1].push_back(0);
            val.push_back(T(200000));
        } else {
            u = leftsize.back();
            leftsize.pop_back();
        }
        return u;
    }
    void del_node(int x) {
        sz[x] = 1,son[0][x] = son[1][x] = 0;
        leftsize.push_back(x);
    }
    fhq_treap(){
        rt = get_node();
        sz[rt] = 0;
    }
    inline int merge(int x,int y){
        if(x*y){
            if(rd[x]<rd[y]){
                son[1][x] = merge(son[1][x],y),maintain(x);
                return x;
            }else{
                son[0][y] = merge(x,son[0][y]),maintain(y);
                return y;
            }
        }else return x|y;
    }
    
    inline void spilt_val(int now,int k,int &x,int &y){
        if(!now) x = y = 0;
        else{
            if(val[now]<=k)x = now,spilt_val(son[1][now],k,son[1][now],y);
            else y = now,spilt_val(son[0][now],k,x,son[0][now]);
            maintain(now);
        }
    }
    inline void spilt_rank(int now,int k,int &x,int &y){
        if(!now) x = y = 0;
        else{
            if(k>sz[son[0][now]])x = now,spilt_rank(son[1][now],k - 1 - sz[son[0][now]],son[1][now],y);
            else y = now,spilt_rank(son[0][now],k,x,son[0][now]);
            maintain(now);
        }
    }
    inline void maintain(int t){
        sz[t] = sz[son[0][t]] + sz[son[1][t]] + 1;
    }
    inline void insert(const T& v){
        int u = get_node();
        val[u] = v;
        int a,b;
        spilt_val(rt,v,a,b);
        rt = merge(merge(a,u),b);
    }
    inline T del(const T& v){
        int x,y,z,d;
        spilt_val(rt,v - 1,x,y);
        spilt_rank(y,1,y,z);
        d = y;
        y = merge(son[0][y],son[1][y]);
        rt = merge(x,merge(y,z));
        del_node(d);
        return val[d];
    }
    inline T pop_front(){
        int x,y,z,d;
        spilt_rank(rt,1,x,y);
        d = x;
        x = merge(son[0][x],son[1][x]);
        rt = merge(x,y);
        del_node(d);
        return val[d];
    }
    inline T pop_back(){
        int x,y,z,d;
        spilt_rank(rt,sz[rt] - 1,x,y);
        d = y;
        y = merge(son[0][y],son[1][y]);
        rt = merge(x,y);
        del_node(d);
        return val[d];
    }
    inline int rank(const T& v){//返回小于v的节点数目
        int a,b,c;
        spilt_val(rt,v - 1,a,b);
        c = sz[a];
        rt = merge(a,b);
        return c;
    }
};

extern fhq_treap<pathlock_node> lockStatus_s[MAP_SIZE * MAP_SIZE][4];
extern fhq_treap<int> lockStatus_e[MAP_SIZE * MAP_SIZE][4]; //锁的状态
extern int lockSize[MAP_SIZE * MAP_SIZE];           //锁的大小
extern bool lockWindows[ROBOT_SIZE];                //临时数组

extern int lockID[MAP_SIZE][MAP_SIZE];         //该格子对应的锁ID
extern int lockCnt;                            //锁的数量
extern char resolve_plat[MAP_SIZE + 2][MAP_SIZE + 2];
extern std::shared_timed_mutex path_mutex;                  //锁的互斥量




// 预处理地图以及对应所变量
void pathlock_init();

// 输出处理后的地图
void printMap();

// 获取锁类型
int pathlock_type(int x,int y);
int pathlock_type(const coordinate2 &pos);

// 判断目标时间窗能否被加锁
bool pathlock_isLockable(int rtidx, const pathlock_node &rhs, int id);

// 释放锁,type = 0 顺序释放锁，type = 1 倒序释放锁
void pathlock_release(int rtidx, int x, int y, int flag = 0);
void pathlock_release(int rtidx, const coordinate2 &pos, int flag = 0);

// 获取锁
bool pathlock_acquire(int rtidx, int x, int y, const pathlock_node& rhs);
bool pathlock_acquire(int rtidx, int x, int y, int s_time, int e_time);
bool pathlock_acquire(int rtidx, const coordinate2& pos, const pathlock_node& time);

// 判断节点在指定时间段是否可经过
bool pathlock_isReachable(int rtidx, int x, int y, int s_time, int e_time);
bool pathlock_isReachable(int rtidx, int x, int y, const pathlock_node& time);


// dijkstra调用时预估的时间
pathlock_node pathlock_getExpectTime(double dd);

// 上锁时预估的时间,size用于表示剩余加入栈的节点数量
pathlock_node pathlock_getExpectTime(double dd , int size);

#endif 