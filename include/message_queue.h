#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <mutex>
#include <deque>
#include <future>

template <typename T> class MessageQueue
{
  public:
    
    //Constructor
    MessageQueue(){};
    
    // Method to send msg to the message queue using move semantics
    void send(T &&msg)
    {
        // Create a lock guard to protect the queue from data race
        std::lock_guard<std::mutex> gLock(_mtx);

        // Push the msg to the back of the queue
        _queue.push_back(std::move(msg));

        // Increment the size of the queue
        _size ++;

        //notify that the element has been added in the queue
        _cond.notify_one();
    }

    // Method to receive msg from the queue using move semantics
    T receive()
    {
        // Create a unique lock to pass it in the wait method of conditional variabe
        std::unique_lock<std::mutex> uLock(_mtx);

        // Check the conditon under lock and than enter the wait based on condition
        _cond.wait(uLock, [this]{ return !_queue.empty(); });   

        // Move the front element from the queue
        T msg = std::move(_queue.front());

        // Remove the front element from the queue
        _queue.pop_front();

        // Increment the size of the queue
        _size --;

        return msg;
    }

    // Method to return True/False based on queue is empty or not
    bool is_empty() 
    {
      std::lock_guard<std::mutex> gLock(_mtx);
      return _queue.empty();
    }

    // Return the current size of the queue
    int get_size() 
    {  
      std::lock_guard<std::mutex> gLock(_mtx);
      return _size; 
    }

  private:

    // FIFO type vector to store the the msgs
    std::deque<T> _queue;

    // Mutex to avoid data race
    std::mutex _mtx;

    // Conditional Varible
    std::condition_variable _cond;

    // Current size of the queue
    int _size{0};
};

#endif
