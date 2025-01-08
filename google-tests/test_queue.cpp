#include <gtest/gtest.h>
#include <functional>
#include <thread>
#include "common/queue.h"  // Include your ThreadSafeQueue header file

// Helper function to simulate worker output and capture messages in the queue
void message_capture_worker(int id, std::function<void(const std::string&)> callback) {
  for (int i = 0; i < 5; ++i) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Message from worker %d: %d\n", id, i);
    callback(buffer);  // Use the callback to write to the queue
    // printf("%s", buffer);  // Print the message for standalone run
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Test that an empty queue initially returns false for canPop
TEST(ThreadSafeQueueTest, PopOnEmptyQueue) {
  ThreadSafeQueue msgQueue;
  std::string message;
  EXPECT_FALSE(msgQueue.canPop(message));
}

// Test that pushing a message results in a non-empty queue
TEST(ThreadSafeQueueTest, PushAndPop) {
  ThreadSafeQueue msgQueue;
  msgQueue.push("test message");
  std::string message;
  EXPECT_TRUE(msgQueue.canPop(message));
  EXPECT_EQ(message, "test message");
}

// Test multiple pushes and pops
TEST(ThreadSafeQueueTest, MultiplePushAndPop) {
  ThreadSafeQueue msgQueue;
  msgQueue.push("message 1");
  msgQueue.push("message 2");
  msgQueue.push("message 3");

  std::string message;
  EXPECT_TRUE(msgQueue.canPop(message));
  EXPECT_EQ(message, "message 1");

  EXPECT_TRUE(msgQueue.canPop(message));
  EXPECT_EQ(message, "message 2");

  EXPECT_TRUE(msgQueue.canPop(message));
  EXPECT_EQ(message, "message 3");

  EXPECT_FALSE(msgQueue.canPop(message));  // Queue should be empty now
}

// Test thread safety by pushing and popping from multiple threads
TEST(ThreadSafeQueueTest, ThreadSafety) {
  ThreadSafeQueue msgQueue;

  auto push_worker = [&msgQueue]() {
    for (int i = 0; i < 10; ++i) {
      msgQueue.push("thread message");
    }
  };

  std::thread t1(push_worker);
  std::thread t2(push_worker);

  t1.join();
  t2.join();

  int count = 0;
  std::string message;
  while (msgQueue.canPop(message)) {
    count++;
  }

  EXPECT_EQ(count, 20);  // Expecting 20 messages in total
}

// Test message capture worker functionality with callback
TEST(ThreadSafeQueueTest, MessageCaptureWorker) {
  ThreadSafeQueue msgQueue;

  auto message_callback = [&](const std::string& msg) { msgQueue.push(msg); };

  std::thread t1(message_capture_worker, 1, message_callback);
  std::thread t2(message_capture_worker, 2, message_callback);

  t1.join();
  t2.join();

  int count = 0;
  std::string message;
  while (msgQueue.canPop(message)) {
    EXPECT_TRUE(message.find("Message from worker") != std::string::npos);
    count++;
  }

  EXPECT_EQ(count, 10);  // Expecting 10 messages in total (5 from each worker)
}
