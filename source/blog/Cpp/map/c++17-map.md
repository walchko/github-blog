---
title: C++17 Map
date: 4 Aug 2019
author: Kevin Walchko
---

- **begin():** Returns an iterator to the first element in the map
- **end():** Returns an iterator to the theoretical element that follows last element in the map
- **size():** Returns the number of elements in the map
- **max_size():** Returns the maximum number of elements that the map can hold
- **empty():** Returns whether the map is empty
- **pair insert(keyvalue, mapvalue):** Adds a new element to the map
- **erase(iterator position):** Removes the element at the position pointed by the iterator
- **erase(const g):** Removes the key value `g` from the map
- **clear():** Removes all the elements from the map

```cpp
#include <map>
#include <iostream>

using namespace std;

int get(const int& k){
    std::map<char, int>::iterator it;
    it = db.find(k);
    if (it == db.end()) printf("error: %d\n", k);
    return it->second;
}

int main(){
    map<char, int> pack = {{'a',3},{'b',55},{'n',66}};
    pack['f'] = 234;
    db.erase('n');
    
    return 0;
}
```

# Reference

- [std::map<A, B>](https://www.geeksforgeeks.org/map-associative-containers-the-c-standard-template-library-stl/)
