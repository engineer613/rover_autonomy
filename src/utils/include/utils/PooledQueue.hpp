#ifndef UTILS_POOLEDQUEUE_HPP
#define UTILS_POOLEDQUEUE_HPP

#include <iostream>
#include <array>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstddef>
#include <thread>

template <typename T, size_t capacity>
class PooledQueue {
public:
   PooledQueue() {
     for (size_t i = 0; i < capacity; ++i) {
       free_indices_[i] = i;
     }
     std::cout << "Initialized Pooled Queue with " << capacity << " buffers" << std::endl;
   }

   T* getBuffer() {
     // LIFO
     std::unique_lock<std::mutex> lock(pool_mutex_);
     buffer_available_.wait(lock, [this] {return num_available_buffers_ > 0;});

     --num_available_buffers_;
     auto top_idx = free_indices_[num_available_buffers_];
     return &pool_[top_idx];
   }

   bool releaseBuffer(T* p_buffer) {
     std::lock_guard<std::mutex> lock(pool_mutex_);
     auto top_idx = p_buffer - &pool_[0];

     if (top_idx >= capacity) {
       std::cerr << "Too many calls to release buffer. Invalid buffer index: "
                 << top_idx << std::endl;
       return false;
     }

     free_indices_[num_available_buffers_] = top_idx;
     ++num_available_buffers_;
     buffer_available_.notify_one();
     return true;
   }

   bool pushToQueue(T* p_buffer) {
     std::lock_guard<std::mutex> lock(queue_mutex_);
     if (queue_size_ < capacity) {
       queue_[queue_back_] = p_buffer;
       queue_back_ = (queue_back_ + 1) % capacity;
       ++queue_size_;
       new_data_available_.notify_one();
       return true;
     }
     return false;
   }

   bool popQueue(T*& p_buffer) {
     std::unique_lock<std::mutex> lock(queue_mutex_);
     new_data_available_.wait(lock, [this] {return queue_size_ > 0 || stop_queueing_ ;});

     if (queue_size_ == 0)  {
       p_buffer = nullptr;
       return false;
     }

     p_buffer = std::move(queue_[queue_front_]);
     queue_front_ = (queue_front_ + 1) % capacity;
     --queue_size_;
     return true;
   }

   void stopQueueing() { stop_queueing_ = true; }
   bool isQueueing() { return !stop_queueing_; }

private:
  size_t num_available_buffers_{capacity};
  std::array<T, capacity> pool_;
  std::array<size_t, capacity> free_indices_;

  std::mutex pool_mutex_;
  std::condition_variable buffer_available_;

  std::array<T*, capacity> queue_;
  std::atomic<bool> stop_queueing_{false};
  std::mutex queue_mutex_;
  std::condition_variable new_data_available_;
  size_t queue_front_;
  size_t queue_back_;
  size_t queue_size_;
};



#endif // UTILS_POOLEDQUEUE_HPP
