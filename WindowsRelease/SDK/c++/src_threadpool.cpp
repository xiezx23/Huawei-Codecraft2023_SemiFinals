#include "inc_threadpool.hpp"

void safeNum::add(int n) {
    c_mutex.lock();
    num += n;
    c_mutex.unlock();
}
void safeNum::sub(int n) {
    c_mutex.lock();
    num -= n;
    c_mutex.unlock();
}
int safeNum::size() {
    return num;
}
bool safeQueue::empty() {
    std::unique_lock<std::mutex> lock(m_mutex);
    bool ret = m_queue.empty();
    return ret;
}
void safeQueue::push(threadTask& task) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(task);
} 
bool safeQueue::dequeue(threadTask& task) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
        return false;
    }
    task = std::move(m_queue.front());
    m_queue.pop();
    return true;
} 

void* threadPool::threadWork(void *arg) {
    threadPool* tpPtr = (threadPool*)arg;
    bool haveWork = false;
    while (!tpPtr->close) {
        threadTask todo;
        {   // 互斥锁保护区域
            std::unique_lock<std::mutex> lock(tpPtr->tCond_mutex);
            // 任务队列空就阻塞线程
            if (tpPtr->taskQueue.empty()) {
                // cerr << "thread sleep...\n";
                tpPtr->runningSize.sub();
                // cerr << "alive thread num:" << tpPtr->runningSize.size() << "\n";
                if (tpPtr->runningSize.size() == 0) tpPtr->mainThreadCond.notify_one();
                tpPtr->tCond.wait(lock);
                tpPtr->runningSize.add();
            }
            // 从任务队列选择任务执行
            // cerr << "thread awake!\n";
            haveWork = tpPtr->taskQueue.dequeue(todo);
        }
        if (haveWork) todo.work(todo.arg);
    }
    tpPtr->runningSize.sub();
    return NULL;
}
threadPool::threadPool(int threadNum) {
    threadSize = threadNum;
    runningSize.add(threadSize);
    tid = new pthread_t[threadSize];
    for (int i = 0; i < threadSize; ++i) {
        pthread_create(&tid[i], NULL, threadWork, (void *)this);
    }
}
void threadPool::addWork(void (*work)(void*), void* arg) {
    threadTask task(work, arg);
    bool mark = false;
    {
        std::unique_lock<std::mutex> lock(tCond_mutex);
        mark = taskQueue.empty();
        taskQueue.push(task);
    }
    if (mark) {
        tCond.notify_one();
    }
}
void threadPool::waitFinish() {
    std::unique_lock<std::mutex> rlock(runningSize.c_mutex);
    while(runningSize.num > 0 || !taskQueue.empty()) {
        // cerr << "wait for branch thread\n";
        mainThreadCond.wait(rlock);
    }
    // cerr << "all works are finished\n";
}

void threadPool::exit() {
    close = true;
    tCond.notify_all();
    for (int i = 0; i < threadSize; ++i) {
        pthread_join(tid[i], NULL);
    }
}