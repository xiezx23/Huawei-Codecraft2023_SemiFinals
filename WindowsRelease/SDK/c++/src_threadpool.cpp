#include "inc_threadpool.hpp"

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
                // cerr << "thread sleep...\n";
                tpPtr->tCond.wait(lock);
            }
            // 从任务队列选择任务执行
            // cerr << "thread awake!\n";
            haveWork = tpPtr->taskQueue.dequeue(todo);
        }
        if (haveWork) todo.work(todo.arg);
    }
    return NULL;
}
threadPool::threadPool(int threadNum) {
    threadSize = threadNum;
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
void threadPool::exit() {
    close = true;
    tCond.notify_all();
    for (int i = 0; i < threadSize; ++i) {
        pthread_join(tid[i], NULL);
    }
}