/**
 * @file ring_buffer.hpp
 * @brief Thread-safe ring buffer for data logging and communication
 */

#pragma once

#include <array>
#include <cstddef>
#include <atomic>
#include <cstring>

namespace omni::utils {

/**
 * @brief Lock-free single-producer single-consumer ring buffer
 *
 * @tparam T Element type
 * @tparam N Buffer capacity (must be power of 2)
 */
template<typename T, size_t N>
class RingBuffer {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");

public:
    RingBuffer() : head_(0), tail_(0) {}

    /**
     * @brief Push element to buffer
     * @param item Item to push
     * @return true if successful, false if buffer full
     */
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t nextHead = (head + 1) & (N - 1);

        if (nextHead == tail_.load(std::memory_order_acquire)) {
            return false;  // Buffer full
        }

        buffer_[head] = item;
        head_.store(nextHead, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop element from buffer
     * @param item Output item
     * @return true if successful, false if buffer empty
     */
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);

        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // Buffer empty
        }

        item = buffer_[tail];
        tail_.store((tail + 1) & (N - 1), std::memory_order_release);
        return true;
    }

    /**
     * @brief Peek at front element without removing
     * @param item Output item
     * @return true if successful, false if buffer empty
     */
    bool peek(T& item) const {
        size_t tail = tail_.load(std::memory_order_relaxed);

        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }

        item = buffer_[tail];
        return true;
    }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief Check if buffer is full
     */
    bool full() const {
        size_t nextHead = (head_.load(std::memory_order_acquire) + 1) & (N - 1);
        return nextHead == tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get number of elements in buffer
     */
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & (N - 1);
    }

    /**
     * @brief Get buffer capacity
     */
    constexpr size_t capacity() const { return N - 1; }

    /**
     * @brief Clear the buffer
     */
    void clear() {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }

    /**
     * @brief Get available space
     */
    size_t available() const {
        return capacity() - size();
    }

private:
    std::array<T, N> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};

/**
 * @brief Simple non-thread-safe ring buffer
 *
 * More efficient when thread safety is not required.
 */
template<typename T, size_t N>
class SimpleRingBuffer {
public:
    SimpleRingBuffer() : head_(0), tail_(0), count_(0) {}

    bool push(const T& item) {
        if (count_ >= N) return false;

        buffer_[head_] = item;
        head_ = (head_ + 1) % N;
        count_++;
        return true;
    }

    bool pop(T& item) {
        if (count_ == 0) return false;

        item = buffer_[tail_];
        tail_ = (tail_ + 1) % N;
        count_--;
        return true;
    }

    bool peek(T& item) const {
        if (count_ == 0) return false;
        item = buffer_[tail_];
        return true;
    }

    T& operator[](size_t index) {
        return buffer_[(tail_ + index) % N];
    }

    const T& operator[](size_t index) const {
        return buffer_[(tail_ + index) % N];
    }

    bool empty() const { return count_ == 0; }
    bool full() const { return count_ >= N; }
    size_t size() const { return count_; }
    constexpr size_t capacity() const { return N; }

    void clear() {
        head_ = tail_ = count_ = 0;
    }

    /**
     * @brief Push, overwriting oldest if full
     */
    void pushOverwrite(const T& item) {
        if (count_ >= N) {
            tail_ = (tail_ + 1) % N;
            count_--;
        }
        push(item);
    }

private:
    std::array<T, N> buffer_;
    size_t head_;
    size_t tail_;
    size_t count_;
};

}  // namespace omni::utils
