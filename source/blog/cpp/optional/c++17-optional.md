---
title: C++17 Optional Keyword
date: 12 Aug 2019
image: "https://i.pinimg.com/564x/61/83/80/618380d9b871fa6f9a9af1feb1b0b7e9.jpg"
---

```cpp
#include <optional>
#include <iostream>

using namespace std;

optional<int> func(){
    return optional<int>();
}

int main(){
    auto i = func();
    cout << i.auto() << endl;        // compiler takes care of it
    cout << i.value_or(21) << endl;  // default if nothing returnd

    return 0;
}
```
