//
// Created by peter on 22/11/24.
//

#include <vector>

template <class item_t>
class Queue {
 public:
  Queue(int size) : mData(size), m_size(size), mHead(0), mTail(0), mItemCount(0) {
    clear();
  }

  int size() {
    return mItemCount;
  }

  bool empty() {
    return mItemCount == 0;
  }

  void clear() {
    mHead = 0;
    mTail = 0;
    mItemCount = 0;
  }

  void add(item_t item) {
    mData[mTail] = item;
    ++mTail;
    ++mItemCount;
    if (mTail > m_size) {
      mTail -= m_size;
    }
  }

  void push(item_t item) {
    add(item);
  }

  item_t head() {
    item_t result = mData[mHead];
    ++mHead;
    if (mHead > m_size) {
      mHead -= m_size;
    }
    --mItemCount;
    return result;
  }

  item_t peek() {
    return mData[mHead];
  }

 protected:
  std::vector<item_t> mData;
  int m_size;
  int mHead = 0;
  int mTail = 0;
  int mItemCount = 0;

 private:
  // while this is probably correct, prevent use of the copy constructor
  Queue(const Queue<item_t> &rhs) {
    //    void(*rhs);
  }
};
