//
// Created by forlinx on 2025/12/17.
//
/*
 * @brief 消息队列类
 * @details 用于在多线程环境中安全地传递消息。
 */
/* 解释整个文件的作用
 * 该文件实现了消息队列类的定义，包括构造函数、推送消息和弹出消息函数。
 * 构造函数用于初始化消息队列，推送消息函数用于将消息放入队列，弹出消息函数用于从队列中取出消息。
 */
#ifndef ENERGYSTORAGE_MSG_QUEUE_H
#define ENERGYSTORAGE_MSG_QUEUE_H

#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class MsgQueue {
public:
    void push(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(msg);
        }
        cv_.notify_one();
    }

    void push(T&& msg) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(std::move(msg));
        }
        cv_.notify_one();
    }

    // 阻塞等待
    T pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [&] { return !queue_.empty(); });
        T msg = std::move(queue_.front());
        queue_.pop();
        return msg;
    }

    // 非阻塞
    bool tryPop(T& out) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop();
        return true;
    }

private:
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
};



#endif //ENERGYSTORAGE_MSG_QUEUE_H