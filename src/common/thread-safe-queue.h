//
// Created by peter on 22/11/24.
//

#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H
#include <mutex>
#include <queue>
#include <string>

class ThreadSafeQueue {
 public:
  void push(const std::string& message) {
    std::lock_guard<std::mutex> lock(mMutex);
    mQueue.push(message);
  }

  bool tryPop(std::string& message) {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mQueue.empty())
      return false;
    message = mQueue.front();
    mQueue.pop();
    return true;
  }

 private:
  std::queue<std::string> mQueue;
  std::mutex mMutex;
};

#endif  // THREAD_SAFE_QUEUE_H
