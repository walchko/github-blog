---
title: Constants in C++
date: 25 Aug 2019
---

- **Bad:** `#define my_value 300.0`
    - This is bad because it is done at compile time, no type checking can be done
    until the compiler starts and you may not catch the error you wanted this to
    be an `int` and not a `double`
    - These are global within your project and can cause all kinds of problems
    - This is an old C solution and needs to go the way of the dinosaur
- **Good:** there are 2 kinds of constants
    - `const` is a run type constant that is define during execution
    - `constexpr` is a compile time constant

```cpp
// forget the fact that all of these variables are called bob and that is an issue!

#define bob 27          // bad, no type checking, global, done at compile-time
const int bob = 27;     // bob created at run-time, then changed to 27
const int bob {27};     // bob initialized at run-time as 27
constexpr int bob {27}; // bob initialized at compile-time to 27

bob = 30;  // illegal since bob is a const/constexpr
```
Since `const` and `constexpr` follow the normal scoping, degubber can follow, there is less of a
chance you will accidentally clobber something else in your code base.

```cpp
// constants.h
#pragma once

// define your own namespace to hold constants
namespace constants
{
    constexpr double pi { 3.14159 };
    constexpr double avogadro { 6.0221413e23 };
    constexpr double my_gravity { 9.81 }; // m/s^2 -- gravity
    // ... other related constants
}
```

## References

- [Constant Expressions and Symbolic Constants](https://www.learncpp.com/cpp-tutorial/const-constexpr-and-symbolic-constants/)
