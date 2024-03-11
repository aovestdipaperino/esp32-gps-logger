#include "CircularBuffer.h"


CircularBuffer::CircularBuffer() :  head(0), tail(0), full(false) {
}


// bool CircularBuffer::saveToFile(const std::string& filename) const {
//   std::ofstream file(filename);
//   if (!file.is_open()) {
//     return false;
//   }

//   // Write full flag
//   file << (full ? 1 : 0) << std::endl;

//   // Write head and tail indices (adjusted for wrapping)
//   size_t adjusted_head = (head + capacity - tail) % capacity;
//   file << adjusted_head << " " << tail << std::endl;

//   // Write buffer data
//   for (size_t i = 0; i < capacity; ++i) {
//     size_t index = (head + i) % capacity;
//     file << data[index] << std::endl;
//   }

//   file.close();
//   return true;
// }


// bool CircularBuffer::loadFromFile(String filename) {
//   std::ifstream file(filename);
//   if (!file.is_open()) {
//     return false;
//   }

//   if (!file) {
//     return false;
//   }


//   // Read full flag
//   int full_int;
//   file >> full_int;
//   full = full_int == 1;

//   // Read head and tail indices (adjusted for wrapping)
//   size_t adjusted_head, tail;
//   file >> adjusted_head >> tail;
//   head = (capacity - adjusted_head + tail) % capacity;

//   // Read buffer data
//   for (size_t i = 0; i < capacity; ++i) {
//     size_t index = (head + i) % capacity;
//     file >> data[index];
//   }

//   file.close();
//   return true;
// }


bool CircularBuffer::push(const String& element) {
  if (full) {
    return false;
  }
  data[tail] = element;
  tail = (tail + 1) % capacity;
  full = size() == capacity;
  return true;
}

String CircularBuffer::pop() {
  if (empty()) {
    throw std::runtime_error("Circular buffer underflow");
  }
  String element = data[head];
  head = (head + 1) % capacity;
  full = false;
  return element;
}

const String CircularBuffer::front() const {
  if (empty()) {
    throw std::runtime_error("Circular buffer is empty");
  }
  return data[head];
}

// // Destructor to free allocated memory (not shown previously)
// template <typename T>
// CircularBuffer<T>::~CircularBuffer() {
//   delete data_;
// }