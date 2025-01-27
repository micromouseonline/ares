//
// Created by peter on 14/01/25.
//

#pragma once

#include <mutex>
#include <string>
#include <vector>
#include "common/queue.h"

class LineProcessor {
 public:
  LineProcessor() : buffer() {
  }

  std::string getBuffer() const {
    return buffer;
  }

  int processQueue(Queue<char>& queue, std::vector<std::string>& log) {
    int count = 0;
    while (!queue.empty()) {
      char ch = queue.head();
      buffer += ch;  // Append character to the buffer
      if (ch == '\0' && buffer.size() > 1 && buffer[buffer.size() - 2] == '\n') {
        // Remove the newline and null characters from the string
        buffer.erase(buffer.size() - 2, 2);
        // Add the complete line to the log
        log.push_back(buffer);
        count++;
        // Clear the buffer for the next line
        buffer.clear();
      }
    }
    return count;
  }

 private:
  std::string buffer;
};
