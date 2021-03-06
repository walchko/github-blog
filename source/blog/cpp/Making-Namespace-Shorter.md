---
title: Making C++ Namespaces Easier to Work With
date: 4 Apr 2019
image: "https://i.pinimg.com/564x/df/e1/5c/dfe15ce2dc32cac67d037455bceb241d.jpg"
image-height: "300px"
---

```cpp
#include <iostream>

namespace foo {
    namespace bar {
         namespace baz {
             int qux = 42;
         }
    }
}

namespace fbz = foo::bar::baz;  // doesn't pollute namespace like using does

int main()
{
    std::cout << fbz::qux << '\n';
}
```

Why is this better than using `using namespace foo::bar::baz`? Well, if you put
it in a header file for a library, you can pollute other cpp files that call
your library and give someone a confusing error. For example, if I always use
`std::cout` but accidentally use a variable `cout` for something else (ok, this
would be a dumb variable or object name but I just want to make a point), the
compiler would think I am using the stream opperator when I am not. All of this
is because I put `using namespace std` in a header file somewhere in some
library.
