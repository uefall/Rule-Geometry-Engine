#pragma once

#include <array>
#include <cstddef>
#include <optional>

namespace rge::trajectory {

template <typename T, std::size_t Capacity>
class FixedRingBuffer {
public:
    static constexpr std::size_t capacity = Capacity;

    bool empty() const { return size_ == 0; }
    std::size_t size() const { return size_; }
    bool full() const { return size_ == Capacity; }

    void clear() {
        size_ = 0;
        head_ = 0;
    }

    void push(const T& value) {
        if (size_ < Capacity) {
            data_[size_] = value;
            ++size_;
            return;
        }
        data_[head_] = value;
        head_ = (head_ + 1) % Capacity;
    }

    const T& back() const { return data_[logical_index(size_ - 1)]; }
    T& back() { return data_[logical_index(size_ - 1)]; }

    const T& operator[](std::size_t index) const {
        return data_[logical_index(index)];
    }

    T& operator[](std::size_t index) { return data_[logical_index(index)]; }

    const T* data() const { return data_.data(); }

private:
    std::size_t logical_index(std::size_t index) const {
        if (size_ < Capacity) {
            return index;
        }
        return (head_ + index) % Capacity;
    }

    std::array<T, Capacity> data_{};
    std::size_t size_{0};
    std::size_t head_{0};
};

}  // namespace rge::trajectory
