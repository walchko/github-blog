---
title: Some Cool c++17 Things
date: 11 Aug 2019
abstract: C++17 std::any
image: "https://i.pinimg.com/564x/ce/d6/f0/ced6f0e56e64735257df96ba9ee28d6c.jpg"
---

`std::any` allows you to ignore the data type:

```cpp
#include <any>
#include <iostream>

int main()
{
    std::cout << std::boolalpha;

    // any type
    std::any a = 1;
    std::cout << a.type().name() << ": " << std::any_cast<int>(a) << '\n';
    a = 3.14;
    std::cout << a.type().name() << ": " << std::any_cast<double>(a) << '\n';
    a = true;
    std::cout << a.type().name() << ": " << std::any_cast<bool>(a) << '\n';

    // bad cast
    try
    {
        a = 1;
        std::cout << std::any_cast<float>(a) << '\n';
    }
    catch (const std::bad_any_cast& e)
    {
        std::cout << e.what() << '\n';
    }

    // has value
    a = 1;
    if (a.has_value())
    {
        std::cout << a.type().name() << '\n';
    }

    // reset
    a.reset();
    if (!a.has_value())
    {
        std::cout << "no value\n";
    }

    // pointer to contained data
    a = 1;
    int* i = std::any_cast<int>(&a);
    std::cout << *i << "\n";
}
```

# References

- [C++ Reference: std::any](https://en.cppreference.com/w/cpp/utility/any)
