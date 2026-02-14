# Template Metaprogramming

## Variadic Templates

```cpp
#include <cstddef>
#include <iostream>
#include <tuple>
#include <utility>

// Fold expressions (C++17)
template <typename... Args>
auto Sum(Args... args) {
    return (args + ...);  // Unary right fold
}

template <typename... Args>
void Print(Args&&... args) {
    ((std::cout << args << ' '), ...);  // Binary left fold
    std::cout << '\n';
}

// Recursive variadic template
template <typename T>
void Log(T&& value) {
    std::cout << value << '\n';
}

template <typename T, typename... Args>
void Log(T&& first, Args&&... rest) {
    std::cout << first << ", ";
    Log(std::forward<Args>(rest)...);
}

// Parameter pack expansion
template <typename... Types>
struct TypeList {
    static constexpr size_t kSize = sizeof...(Types);
};

template <typename... Args>
auto MakeTupleAdvanced(Args&&... args) {
    return std::tuple<std::decay_t<Args>...>(std::forward<Args>(args)...);
}
```

## SFINAE and if constexpr

```cpp
#include <cstdint>
#include <cstddef>
#include <type_traits>

// SFINAE with std::enable_if (older style)
template <typename T>
std::enable_if_t<std::is_integral_v<T>, T>
DoubleValue(T value) {
    return value * 2;
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T>
DoubleValue(T value) {
    return value * 2.0;
}

// Modern: if constexpr (C++17)
template <typename T>
auto Process(T value) {
    if constexpr (std::is_integral_v<T>) {
        return value * 2;
    } else if constexpr (std::is_floating_point_v<T>) {
        return value * 2.0;
    } else {
        return value;
    }
}

// Detection idiom
template <typename T, typename = void>
struct HasSerialize : std::false_type {};

template <typename T>
struct HasSerialize<T, std::void_t<decltype(std::declval<T>().Serialize())>>
    : std::true_type {};

template <typename T>
constexpr bool kHasSerializeV = HasSerialize<T>::value;

// Use with if constexpr
template <typename T>
void Save(const T& obj) {
    if constexpr (kHasSerializeV<T>) {
        obj.Serialize();
    } else {
        // Default serialization
    }
}
```

## Type Traits

```cpp
#include <type_traits>

// Custom type traits
template <typename T>
struct RemoveAllPointers {
    using type = T;
};

template <typename T>
struct RemoveAllPointers<T*> {
    using type = typename RemoveAllPointers<T>::type;
};

template <typename T>
using RemoveAllPointersT = typename RemoveAllPointers<T>::type;

// Conditional types
template <bool Condition, typename T, typename F>
struct ConditionalType {
    using type = T;
};

template <typename T, typename F>
struct ConditionalType<false, T, F> {
    using type = F;
};

// Compile-time type selection
template <size_t N>
struct BestIntegralType {
    using type = std::conditional_t<N <= 8, uint8_t,
                 std::conditional_t<N <= 16, uint16_t,
                 std::conditional_t<N <= 32, uint32_t, uint64_t>>>;
};

// Check for member functions
template <typename T, typename = void>
struct HasReserve : std::false_type {};

template <typename T>
struct HasReserve<T, std::void_t<decltype(std::declval<T>().reserve(size_t{}))>>
    : std::true_type {};
```

## CRTP (Curiously Recurring Template Pattern)

```cpp
// Static polymorphism with CRTP
template <typename Derived>
class Shape {
public:
    double Area() const {
        return static_cast<const Derived*>(this)->AreaImpl();
    }

    void Draw() const {
        static_cast<const Derived*>(this)->DrawImpl();
    }
};

class Circle : public Shape<Circle> {
    double radius_;
public:
    explicit Circle(double r) : radius_(r) {}

    double AreaImpl() const {
        return 3.14159 * radius_ * radius_;
    }

    void DrawImpl() const {
        std::cout << "Drawing circle\n";
    }
};

class Rectangle : public Shape<Rectangle> {
    double width_, height_;
public:
    Rectangle(double w, double h) : width_(w), height_(h) {}

    double AreaImpl() const {
        return width_ * height_;
    }

    void DrawImpl() const {
        std::cout << "Drawing rectangle\n";
    }
};

// CRTP for mixin capabilities
template <typename Derived>
class Printable {
public:
    void Print() const {
        std::cout << static_cast<const Derived*>(this)->ToString() << '\n';
    }
};

class User : public Printable<User> {
    std::string name_;
public:
    explicit User(std::string name) : name_(std::move(name)) {}

    std::string ToString() const {
        return "User: " + name_;
    }
};
```

## Template Template Parameters

```cpp
#include <deque>
#include <list>
#include <vector>

// Template template parameter
template <typename T, template <typename, typename> class Container>
class Stack {
    Container<T, std::allocator<T>> data_;

public:
    void Push(const T& value) {
        data_.push_back(value);
    }

    T Pop() {
        T value = data_.back();
        data_.pop_back();
        return value;
    }

    size_t Size() const {
        return data_.size();
    }
};

// Usage with different containers
Stack<int, std::vector> vector_stack;
Stack<int, std::deque> deque_stack;
Stack<int, std::list> list_stack;
```

## Compile-Time Computation

```cpp
#include <array>

// Compile-time factorial
constexpr int Factorial(int n) {
    return n <= 1 ? 1 : n * Factorial(n - 1);
}

constexpr int kFact5 = Factorial(5);  // Computed at compile time

// Compile-time prime checking
constexpr bool IsPrime(int n) {
    if (n < 2) return false;
    for (int i = 2; i * i <= n; ++i) {
        if (n % i == 0) return false;
    }
    return true;
}

// Generate compile-time array of primes
template <size_t N>
constexpr auto GeneratePrimes() {
    std::array<int, N> primes{};
    int count = 0;
    int candidate = 2;

    while (count < N) {
        if (IsPrime(candidate)) {
            primes[count++] = candidate;
        }
        ++candidate;
    }

    return primes;
}

constexpr auto kFirst10Primes = GeneratePrimes<10>();
```

## Expression Templates

```cpp
// Lazy evaluation with expression templates
template <typename E>
class VecExpression {
public:
    double operator[](size_t i) const {
        return static_cast<const E&>(*this)[i];
    }

    size_t Size() const {
        return static_cast<const E&>(*this).Size();
    }
};

class Vec : public VecExpression<Vec> {
    std::vector<double> data_;

public:
    explicit Vec(size_t n) : data_(n) {}

    double operator[](size_t i) const { return data_[i]; }
    double& operator[](size_t i) { return data_[i]; }
    size_t Size() const { return data_.size(); }

    // Evaluate expression template
    template <typename E>
    Vec& operator=(const VecExpression<E>& expr) {
        for (size_t i = 0; i < Size(); ++i) {
            data_[i] = expr[i];
        }
        return *this;
    }
};

// Binary operation expression
template <typename E1, typename E2>
class VecSum : public VecExpression<VecSum<E1, E2>> {
    const E1& lhs_;
    const E2& rhs_;

public:
    VecSum(const E1& lhs, const E2& rhs) : lhs_(lhs), rhs_(rhs) {}

    double operator[](size_t i) const {
        return lhs_[i] + rhs_[i];
    }

    size_t Size() const { return lhs_.Size(); }
};

// Operator overload
template <typename E1, typename E2>
VecSum<E1, E2> operator+(const VecExpression<E1>& lhs,
                         const VecExpression<E2>& rhs) {
    return VecSum<E1, E2>(static_cast<const E1&>(lhs),
                          static_cast<const E2&>(rhs));
}

// Usage: a = b + c + d  (no temporaries created!)
```

## Quick Reference

| Technique | Use Case | Performance |
|-----------|----------|-------------|
| Variadic Templates | Variable arguments | Zero overhead |
| SFINAE | Conditional compilation | Compile-time |
| if constexpr | Type-based branching | Zero overhead |
| CRTP | Static polymorphism | No vtable cost |
| Expression Templates | Lazy evaluation | Eliminates temps |
| Type Traits | Type introspection | Compile-time |
| Fold Expressions | Parameter pack ops | Optimal |
| Template Specialization | Type-specific impl | Zero overhead |
