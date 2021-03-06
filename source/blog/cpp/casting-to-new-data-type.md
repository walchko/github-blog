---
title: C++ and Casting to new Data Types
date: 25 Aug 2019
image: "https://i.pinimg.com/564x/22/9b/c7/229bc7a571e01a5416fea15ae4cdc658.jpg"
image-height: "300px"
---

- **static_cast:** implicit conversions, no runtime checks or overhead
- **const_cast:** add or remove `const` to a variable
- **dynamic_cast:** used when doing polymorphism, only used on pointers, and does
runtime checks (can have significant overhead).
- **reinterpret_cast:** very powerful, can change a pointer from one type
to another. Used on variables that are not type-safe and reinterprets their
underlying bit pattern.

## Examples

### const_cast

```cpp
int a = 5; // NOTE: non-const object
const int* pA = &a;
*pA = 10; // compiler error, pA is a pointer to const int
int* pX = const_cast<int*>(pA); // cast away constness
*pX = 10 // fine and a is now 10

const int a = 5; // NOTE: const object
const int* pA = &a;
*pA = 10; // compiler error, pA is a pointer to const int
int* pX = const_cast<int*>(pA); // cast away constness
*pX = 10 // Free ticket to a long journey of UNDEFINED BEHAVIOR

int* a = nullptr;
const char* ptr = "Hello";
a = const_cast<int*>(ptr); // Fail
a = reinterpret_cast<int*>(ptr); // Fail, reinterpret_cast can't cast away const qualifiers
a = reinterpret_cast<int*>(const_cast<char*>(ptr)); // Fine, as long as you know why you are doing it
```

## References

- [Stack Overflow](https://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used)
- [Quora](https://www.quora.com/How-do-you-explain-the-differences-among-static_cast-reinterpret_cast-const_cast-and-dynamic_cast-to-a-new-C++-programmer)
