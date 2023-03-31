#include "inc_threadpool.hpp"

void safeNum::add(int n = 1) {
    c_mutex.lock();
    num += n;
    c_mutex.unlock();
}
void safeNum::sub(int n = 1) {
    c_mutex.lock();
    num -= n;
    c_mutex.unlock();
}
int safeNum::size() {
    return num;
}

bool safeQueue::empty() {
    m_mutex.lock();
    bool ret = m_queue.empty();
    m_mutex.unlock();
    return ret;
}
void safeQueue::push(threadTask& task) {
    m_mutex.lock();
    m_queue.push(task);
    m_mutex.unlock();
} 
bool safeQueue::dequeue(threadTask& task) {
    m_mutex.lock();
    if (m_queue.empty()) {
        m_mutex.unlock();
        return false;
    }
    task = std::move(m_queue.front());
    m_queue.pop();
    m_mutex.unlock();
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
                cerr << "thread sleep...\n";
                tpPtr->runningSize.sub();
                if (tpPtr->runningSize.size() == 0) tpPtr->mainThreadCond.notify_one();
                tpPtr->tCond.wait(lock);
                tpPtr->runningSize.add();
            }
            // 从任务队列选择任务执行
            cerr << "thread awake!\n";
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
    while(runningSize.size() > 0 || !taskQueue.empty()) {
        std::unique_lock<std::mutex> lock(mTrea_mutex);
        cerr << "wait for branch threads\n";
        mainThreadCond.wait(lock);
    }
    cerr << "all works are finished\n";
}
void threadPool::exit() {
    close = true;
    tCond.notify_all();
    for (int i = 0; i < threadSize; ++i) {
        pthread_join(tid[i], NULL);
    }
}