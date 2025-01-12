#include <functional>
#include <iostream>

struct Data {
  int x;
  int y;
};

typedef std::function<Data(int)> DataCallbackFunction;

//////////////////////////////////////////////////////////////////////////
class Worker {
 public:
  void setCallback(DataCallbackFunction cb) {
    callback = cb;
  }

  void getData(int q) {
    if (callback) {
      Data data = callback(q);
      std::cout << "Data received: x = " << data.x << ", y = " << data.y << std::endl;
    }
  }

 private:
  DataCallbackFunction callback;
};
//////////////////////////////////////////////////////////////////////////
class Manager {
 public:
  Manager() {
    DataCallbackFunction cb = [this](int q) { return this->dataCallback(q); };
    worker.setCallback(cb);
  }

  Data dataCallback(int q) {
    return {2 * q, 3 * q};
  }

  void requestData(int q) {
    worker.getData(q);
  }

 private:
  Worker worker;
};

//////////////////////////////////////////////////////////////////////////
int main() {
  Manager manager;
  manager.requestData(5);  // This should output "Data received: x = 10, y = 15"
  return 0;
}
