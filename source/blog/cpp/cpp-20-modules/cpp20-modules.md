---
title: C++20 Modules
date: 4 June 2021
image: header.jpg
---

With modules, you can finally create *one* file that holds
both the declaration along with the implementation!

- No `#ifndef` or `#pragma once` macro needed, module is only
imported *once*
- all variables are private
- macOS requires the file extension to be `*.cppm` for a 
module

```c++
// hello.cppm
export module helloworld;  // module declaration
import <iostream>;         // import declaration
 
export void hello() {      // export declaration
    std::cout << "Hello world!\n";
}
```

```c++
// main.cpp
import helloworld;  // import declaration
 
int main() {
    hello();
}
```

# References

- [cppreference: Modules](https://en.cppreference.com/w/cpp/language/modules)
