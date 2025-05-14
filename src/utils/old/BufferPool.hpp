#ifndef GPIO_BUFFERPOOL_HPP
#define GPIO_BUFFERPOOL_HPP

#include <array>
#include <mutex>
#include <cstddef>
#include <thread>
#include <condition_variable>

template<typename T, size_t size>
class BufferPool {
public:
  BufferPool() {
    for (size_t i = 0; i < size; i++) {
      free_indices_[i] =  i;
    }
    std::cout << "[BufferPool] Initialized Buffer Pool with " << size << " buffers" << std::endl;
  }

  T* get() {
    std::unique_lock<std::mutex> lock(pool_mutex_);
    // Makes sure there is a buffer available,
    // so it doesn't return a null pointer
    cv_.wait(lock, [this] {return available_ > 0;});

    --available_;
    auto top_idx = free_indices_[available_];
    return &pool_[top_idx];
  }

  bool release(T* buffer_ptr) {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    auto top_idx = buffer_ptr - &pool_[0];

    if (available_ >= size) {
      std::cout << "[BufferPool] Error: Buffer pool is full. Multiple Calls to release" << std::endl;
      return false;
    }

    free_indices_[available_] = top_idx;
    available_++;
    cv_.notify_one(); // Notify one thread waiting for buffer
    //std::cout << "[BufferPool] Buffer released" << std::endl;
    return true;
  }

private:
  size_t available_{size};
  std::array<T, size> pool_;
  std::array<size_t, size> free_indices_;

  std::mutex pool_mutex_;
  std::condition_variable cv_;
};

#endif // GPIO_BUFFERPOOL_HPP
