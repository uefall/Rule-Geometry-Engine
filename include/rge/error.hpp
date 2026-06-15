#pragma once

#include <optional>
#include <type_traits>

namespace rge {

enum class GeomError {
    kOk = 0,
    kDegenerateInput,
    kBufferTooSmall,
    kNonSimplePolygon,
    kInsufficientHistory,
    kInvalidArgument,
};

template <typename T>
class Result {
public:
    Result(T value) : value_(std::move(value)), error_(GeomError::kOk) {}

    static Result ok(T value) { return Result(std::move(value)); }

    static Result err(GeomError error) {
        Result result;
        result.error_ = error;
        return result;
    }

    bool ok() const { return error_ == GeomError::kOk; }
    explicit operator bool() const { return ok(); }

    GeomError error() const { return error_; }

    const T& value() const { return *value_; }
    T& value() { return *value_; }

    T value_or(T fallback) const {
        return ok() ? *value_ : fallback;
    }

private:
    Result() = default;

    std::optional<T> value_;
    GeomError error_{GeomError::kOk};
};

template <>
class Result<void> {
public:
    static Result success() { return Result(GeomError::kOk); }

    static Result err(GeomError error) { return Result(error); }

    bool ok() const { return error_ == GeomError::kOk; }
    explicit operator bool() const { return ok(); }

    GeomError error() const { return error_; }

private:
    explicit Result(GeomError error) : error_(error) {}

    GeomError error_{GeomError::kOk};
};

}  // namespace rge
