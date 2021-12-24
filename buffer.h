#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <mutex>

/**
 * \tparam Type Type of elements in this ring buffer.
 * \tparam len Max length of this ring buffer.
 * \attention Length should be 2^N.
 */
template<typename Type, unsigned int len>
class Buffer {
private:
    Type data_[len];
    unsigned int head_;
    unsigned int tail_;
    std::mutex lock_[len];
    std::mutex head_lock_;
    const unsigned int and_to_mod_ = len - 1;

public:
    Buffer<Type, len>() : head_(0), tail_(0) {
        assert(len);
        assert(!(len & (len - 1)));
    }

    ~Buffer() = default;

    [[maybe_unused]] [[nodiscard]] inline unsigned int Size() const {
        return len;
    }

    [[maybe_unused]] [[nodiscard]] inline unsigned int Load() {
        return (tail_ - head_ + len) & and_to_mod_;
    }

    [[maybe_unused]] [[nodiscard]] inline bool Empty() const {
        return head_ == tail_;
    }

    inline void Push(const Type &obj) {
        std::lock_guard<std::mutex> lock(lock_[tail_]);
        data_[tail_] = obj;
        ++tail_;
        tail_ &= and_to_mod_;

        if (head_ == tail_) {
            std::lock_guard<std::mutex> head_lock(head_lock_);
            ++head_;
            head_ &= and_to_mod_;
        }
    }

    inline bool Pop(Type &obj) {
        if (head_ == tail_)
            return false;
        std::lock_guard<std::mutex> lock(lock_[head_]);
        obj = data_[head_];
        std::lock_guard<std::mutex> head_lock(head_lock_);
        ++head_;
        head_ &= and_to_mod_;
        return true;
    }

    [[maybe_unused]] Type &operator[](unsigned int id) {
        while (tail_ + id < 0) id += len;
        return data_[(tail_ + id) & and_to_mod_];
    }
};

#endif  // _BUFFER_H_
