#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <fstream>
#include <Arduino.h>

class CircularBuffer {
public:
  // Constructor with capacity
  CircularBuffer();
  //~CircularBuffer();

  // Check if empty
  bool empty() const { return head == tail && !full; }

  // Check if full
  bool isFull() const { return full; }

  // Get current size
  size_t size() const { return full ? capacity : (tail - head + capacity) % capacity; }

  // Push element to the back. Returns true if it succeeds.
  bool push(const String& element);

  // Pop element from the front
  String pop();

  // Peek at the front element without removing
  const String front() const;

  // // Save buffer state to a file
  // bool saveToFile(String filename) const;

  // // Load buffer state from a file
  // bool loadFromFile(String filename);

private:
  const size_t capacity = 50;
  size_t head;
  size_t tail;
  bool full;
  String data[50];
};

#endif