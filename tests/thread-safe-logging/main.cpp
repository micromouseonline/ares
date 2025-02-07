#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>

/***
 * In this example, we have a worker process running a separate thread.
 *
 * I want it to be able to pass messages to a main thread via a queue.
 * The queue runs in the the main thread and it is processed by an
 * observer sunning in the same thread.
 *
 * Here, the worker passes log messages. Because the worker sends copies
 * of local data, there is no need for a mutex in the worker and it is
 * unaware that it runs in a different thread.
 *
 * The std::this_thread::sleep_for() call is just a way to get a delay.
 *
 * It is the queue that is protected with a mutex lock.
 *
 * We can also use this mechanism to sent Robot state updates to the main
 * thread.
 */
struct LogMessage {
  std::string timestamp;
  std::string level;
  std::string message;
};

/////////////////////////////////////////////////////////////////////////////////
template <typename T>
class ThreadSafeQueue {
 public:
  void push(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
    cond_var_.notify_one();
  }

  bool pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_var_.wait(lock, [this] { return !queue_.empty() || stop_flag_; });
    if (queue_.empty()) {
      return false;
    }
    value = queue_.front();
    queue_.pop();
    return true;
  }

  bool shouldStop() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_flag_;
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_flag_ = true;
    cond_var_.notify_all();
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable cond_var_;
  bool stop_flag_ = false;
};

using LogQueueRef = ThreadSafeQueue<LogMessage>&;
/////////////////////////////////////////////////////////////////////////////////

class Worker {
 public:
  void run(LogQueueRef logQueue) {
    for (int i = 0; i < 10; ++i) {
      // Simulate robot behavior
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      logMessage(logQueue, "INFO", "Simulation step " + std::to_string(i));
    }
    logMessage(logQueue, "FINISHED", "Simulation stopped");
    // Signal the observer to stop after the last log message
    // in other contexts, we might not bother to do this but
    // without it, in this case, the observer will not exit
    logQueue.stop();
  }

 private:
  void logMessage(LogQueueRef logQueue, const std::string& level, const std::string& message) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
    LogMessage log = {oss.str(), level, message};
    logQueue.push(log);
  }
};

/////////////////////////////////////////////////////////////////////////////////
class MainThreadObserver {
 public:
  MainThreadObserver(LogQueueRef logQueue)
      : logQueue_(logQueue) {
  }

  void run() {
    LogMessage log;
    while (!logQueue_.shouldStop() || !logQueue_.pop(log)) {
      if (logQueue_.pop(log)) {
        displayLog(log);
      }
    }
    // Drain any remaining logs after the stop flag is set
    while (logQueue_.pop(log)) {
      displayLog(log);
    }
  }

 private:
  void displayLog(const LogMessage& log) {
    std::cout << "[" << log.timestamp << "] [" << log.level << "] " << log.message << std::endl;
  }

  LogQueueRef logQueue_;
};

/////////////////////////////////////////////////////////////////////////////////

int main() {
  ThreadSafeQueue<LogMessage> logQueue;
  Worker worker;
  MainThreadObserver observer(logQueue);

  std::thread worker_thread(&Worker::run, &worker, std::ref(logQueue));

  observer.run();

  std::cout << "Observer finished" << std::endl;

  worker_thread.join();
  return 0;
}
