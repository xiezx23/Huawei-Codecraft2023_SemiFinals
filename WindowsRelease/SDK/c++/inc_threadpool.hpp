#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP
#include "inc_codecraft2023.hpp"
/********************************
 *  <假设线程池名字 tpName, 线程数为 threadSize>
 * 
 *  声明线程池: threadPool* tpName = new threadPool(threadSize); 
 *  销毁线程池: tpName->exit();
 *  加入任务:   tpName->addWork(void* (void*), void*);
 *  
 *  示例代码
void hello(void* a) {
    cout << "new function " << *(int*)a << endl;
}
int main() {
    threadPool* tp = new threadPool(4);
    int a = 11;
    tp->addWork(&hello, (void*)&a);
    tp->exit();
    return 0;
}
********************************/

// 封装调用函数，work为函数指针，arg为形参
struct threadTask {
    std::function<void(void*)> work;
    void* arg;
    threadTask() {}
    threadTask(std::function<void(void*)> _work, void* _arg):work(_work),arg(_arg){}
};

// 并行安全的队列
struct safeQueue {
private:
    std::mutex m_mutex;
    std::queue<threadTask> m_queue;
    
public:
    bool empty();
    void push(threadTask& task);
    bool dequeue(threadTask& task);
};

// 线程池
struct threadPool {
private:
    int threadSize;                 // 线程数
    pthread_t* tid;                 // 维护tid
    bool close = false;             // 关闭开关
    safeQueue taskQueue;            // 任务队列 
    std::mutex tCond_mutex;         // 休眠锁
    std::condition_variable tCond;  // 条件变量
    static void* threadWork(void *arg); // 线程回调函数

public:
    threadPool() {}
    threadPool(int threadNum);
    ~threadPool() {delete(tid);}
    void addWork(void (*work)(void*), void* arg);
    void exit();
};

#endif