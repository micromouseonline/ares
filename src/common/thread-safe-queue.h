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
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push(message);
  }

  bool tryPop(std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty())
      return false;
    message = m_queue.front();
    m_queue.pop();
    return true;
  }

  bool empty() {
    return m_queue.empty();
  }

 private:
  std::queue<std::string> m_queue;
  std::mutex m_mutex;
};

#endif  // THREAD_SAFE_QUEUE_H
