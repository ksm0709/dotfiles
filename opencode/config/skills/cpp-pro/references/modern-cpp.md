# Modern C++20/23 Features

## Concepts and Constraints

```cpp
#include <concepts>
#include <functional>
#include <iostream>

// Define custom concepts
template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <typename T>
concept Hashable = requires(T a) {
    { std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
};

template <typename T>
concept Container = requires(T c) {
    typename T::value_type;
    typename T::iterator;
    { c.begin() } -> std::same_as<typename T::iterator>;
    { c.end() } -> std::same_as<typename T::iterator>;
    { c.size() } -> std::convertible_to<std::size_t>;
};

// Use concepts for function constraints
template <Numeric T>
T Add(T a, T b) {
    return a + b;
}

// Concept-based overloading
template <std::integral T>
void Process(T value) {
    std::cout << "Processing integer: " << value << '\n';
}

template <std::floating_point T>
void Process(T value) {
    std::cout << "Processing float: " << value << '\n';
}
```

## Ranges and Views

```cpp
#include <algorithm>
#include <ranges>
#include <vector>

void DemonstrateRanges() {
    std::vector<int> numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    // Filter, transform, take - all lazy evaluation
    auto result = numbers
        | std::views::filter([](int n) { return n % 2 == 0; })
        | std::views::transform([](int n) { return n * n; })
        | std::views::take(3);

    // Copy to vector only when needed
    std::vector<int> materialized(result.begin(), result.end());

    // Custom range adaptor pipeline
    auto is_even = [](int n) { return n % 2 == 0; };
    auto square = [](int n) { return n * n; };
    auto pipeline = std::views::filter(is_even) | std::views::transform(square);

    auto processed = numbers | pipeline;
    (void)materialized;
    (void)processed;
}
```

## Coroutines

```cpp
#include <coroutine>
#include <iostream>

// Generator coroutine
template <typename T>
struct Generator {
    struct promise_type {
        T current_value;

        auto get_return_object() {
            return Generator{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        std::suspend_always yield_value(T value) {
            current_value = value;
            return {};
        }

        void return_void() {}
        void unhandled_exception() { std::terminate(); }
    };

    std::coroutine_handle<promise_type> handle;

    explicit Generator(std::coroutine_handle<promise_type> h) : handle(h) {}
    ~Generator() { if (handle) handle.destroy(); }

    bool MoveNext() {
        handle.resume();
        return !handle.done();
    }

    T CurrentValue() {
        return handle.promise().current_value;
    }
};

// Usage
Generator<int> Fibonacci() {
    int a = 0, b = 1;
    while (true) {
        co_yield a;
        auto next = a + b;
        a = b;
        b = next;
    }
}

struct Task {
    struct promise_type {
        Task get_return_object() {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }
        void return_void() {}
        void unhandled_exception() { std::terminate(); }
    };

    std::coroutine_handle<promise_type> handle;

    explicit Task(std::coroutine_handle<promise_type> h) : handle(h) {}
    ~Task() { if (handle) handle.destroy(); }
};

Task AsyncOperation() {
    std::cout << "Starting async work\n";
    co_await std::suspend_always{};
    std::cout << "Resuming async work\n";
    co_return;
}
```

## Three-Way Comparison (Spaceship)

```cpp
#include <compare>

struct Point {
    int x, y;

    // Auto-generate all comparison operators
    auto operator<=>(const Point&) const = default;
};

// Custom spaceship operator
struct Version {
    int major, minor, patch;

    std::strong_ordering operator<=>(const Version& other) const {
        if (auto cmp = major <=> other.major; cmp != 0) return cmp;
        if (auto cmp = minor <=> other.minor; cmp != 0) return cmp;
        return patch <=> other.patch;
    }

    bool operator==(const Version& other) const = default;
};
```

## Designated Initializers

```cpp
struct Config {
    std::string host = "localhost";
    int port = 8080;
    bool ssl_enabled = false;
    int timeout_ms = 5000;
};

// C++20 designated initializers
Config cfg {
    .host = "example.com",
    .port = 443,
    .ssl_enabled = true
    // timeout_ms uses default
};
```

## Modules (C++20)

```cpp
// math.cppm - module interface
export module math;

export namespace math {
    template <typename T>
    T Add(T a, T b) {
        return a + b;
    }

    class Calculator {
    public:
        int Multiply(int a, int b);
    };
}

// Implementation
module math;

int math::Calculator::Multiply(int a, int b) {
    return a * b;
}

// Usage in other files
import math;

int main() {
    int result = math::Add(5, 3);
    math::Calculator calc;
    int product = calc.Multiply(4, 7);
    return result + product;
}
```

## constexpr Enhancements

```cpp
#include <algorithm>
#include <string>
#include <vector>

// C++20: constexpr std::string and std::vector
constexpr int ComputeAtCompileTime() {
    std::vector<int> vec{1, 2, 3, 4, 5};
    std::ranges::reverse(vec);
    return vec[0];  // Returns 5
}

constexpr int kCompileTimeValue = ComputeAtCompileTime();

// constexpr virtual functions (C++20)
struct Base {
    constexpr virtual int GetValue() const { return 42; }
    constexpr virtual ~Base() = default;
};

struct Derived : Base {
    constexpr int GetValue() const override { return 100; }
};
```

## std::format (C++20)

```cpp
#include <format>
#include <string>

struct Point {
    int x;
    int y;
};

template <>
struct std::formatter<Point> {
    constexpr auto parse(format_parse_context& ctx) {
        return ctx.begin();
    }

    auto format(const Point& p, format_context& ctx) const {
        return std::format_to(ctx.out(), "({}, {})", p.x, p.y);
    }
};

std::string BuildFormattedMessage() {
    std::string msg = std::format("Hello, {}!", "World");
    std::string text = std::format("{1} {0}", "World", "Hello");

    double pi = 3.14159265;
    std::string formatted = std::format("Pi: {:.2f}", pi);  // Pi: 3.14
    std::string point = std::format("Point={}", Point{.x = 3, .y = 4});

    return std::format("{} | {} | {} | {}", msg, text, formatted, point);
}
```

## Quick Reference

| Feature | C++17 | C++20 | C++23 |
|---------|-------|-------|-------|
| Concepts | - | ✓ | ✓ |
| Ranges | - | ✓ | ✓ |
| Coroutines | - | ✓ | ✓ |
| Modules | - | ✓ | ✓ |
| Spaceship | - | ✓ | ✓ |
| std::format | - | ✓ | ✓ |
| std::expected | - | - | ✓ |
| std::print | - | - | ✓ |
| Deducing this | - | - | ✓ |
