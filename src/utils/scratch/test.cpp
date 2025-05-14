#include <iostream>
#include <memory>
#include <string>

#include "PooledQueue.hpp"

PooledQueue<int, 50> queue;

int num_ints = 100;

void producer() {
    int * buffer;
    for (int i = 0; i <=num_ints; i++) {
        while (!(buffer = queue.getBuffer())) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        *buffer = i;
        std::cout << "[Producer] Pushing: " << i << std::endl;
        auto push_ok = queue.pushToQueue(buffer);
    }
}

void consumer() {
    int* p_buffer;
    for (int i = 0; i <= num_ints; i++) {
        auto pop_ok = queue.popQueue(p_buffer);
        std::cout << "Popped: " << *p_buffer << std::endl;
        queue.releaseBuffer(p_buffer);
    }
}


int main()
{
    std::thread p_thread(producer);
    std::thread c_thread(consumer);

    p_thread.join();
    c_thread.join();

    return 0;
}