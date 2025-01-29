//
// Created by peter on 16/12/23.
//

#pragma once
#include <memory>

// from: https://codereview.stackexchange.com/questions/173929/modern-c-singleton-template
template <typename T>
class Singleton {
 public:
  static T* instance() {
    /***
     * using a constructor token to allow the base class
     * to call the subclass's constructor without needing to be a friend.
     */
    static T instance{token{}};
    return &instance;
  }

 protected:
  struct token {};
  Singleton() = default;
  ~Singleton() = default;

 private:
  Singleton& operator=(const Singleton) = delete;
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(Singleton&&) = delete;
};

/***
 * Example of use

class Test final : public Singleton<Test> {
  ///
  /// Although the constructor is public, it can't be called
  /// without a Singleton<T>::token object, meaning that
  /// access to it is now controlled and you do not need
  /// to make declare the singleton class as a friend
  /// ... and no, I don't understand that

 public:
  Test(token) {
    //std::cout << "constructed" << std::endl;
  }

  void use() const {
     std::cout << "in use" << std::endl;
  };

  /// 1: if you need a destructor make it public
  /// 2: you probably need a destructor if you want to test the singleton
  ~Test() {
    std::cout << "Test destructed" << std::endl;
  }
};

 /// in your code do this
 // note we are getting a _reference_ so not need for the pointer dereference
 auto const &test = Test::instance();
 test.use();

*/
