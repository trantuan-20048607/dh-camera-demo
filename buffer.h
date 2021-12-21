#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <mutex>
#include <chrono>

template<typename Type, int len>
class Buffer {
private:
    Type data_[len];
    int head_;
    int tail_;
    std::mutex reading_[len];

public:
    Buffer<Type, len>() : head_(0), tail_(0) {}

    ~Buffer() = default;

    [[maybe_unused]] [[nodiscard]] constexpr int Size() const {
        return len;
    }

    [[nodiscard]] inline bool Empty() const {
        return head_ == tail_;
    }

    void Push(const Type &obj) {
        std::lock_guard<std::mutex> lock(reading_[head_]);
        data_[tail_] = obj;
        tail_ = (tail_ + 1) % len;
        if (head_ == tail_) {
            head_ = (head_ + 1) % len;
        }
    }

    bool Pop(Type &obj) {
        if (Empty())
            return false;
        std::lock_guard<std::mutex> lock(reading_[head_]);
        obj = data_[head_];
        head_ = (head_ + 1) % len;
        return true;
    }

    [[maybe_unused]] Type &operator[](int id) {
        while (tail_ + id < 0) id += len;
        return data_[(tail_ + id) % len];
    }
};

#endif  // _BUFFER_H_
