---
title: C++ Lambdas
date: 3 Aug 2019
image: "https://i.pinimg.com/564x/8a/5c/27/8a5c27e657eb29f380748dc2a37d6d40.jpg"
image-height: "400px"
---

A lambda is an anonomous function of the form: `[capture](parameters){definition}`.

```cpp
int a=0;
vector<int> v = {0,1,2,3,4,5};

// capture: automatically grab the reference of a
// parameters: given an int input called d
// definition: add d to a. Note that no return statement is needed because this is simple
auto ff = [&a](int d){a += d;};

// given 2 iterators, for_each will feed the value into the lambda as an arg
std:for_each(v.begin(),v.end(),ff);

cout << a << endl; // sum(5) => 15
```

```cpp
map<int,int> dd{{1,11},{2,22},{3,33}};

// more complex lambda
// capture: grab dd
// parameters: given a key value
// definition: search for key, return -1 if not found, otherwise return value
auto f = [dd](int key){
    auto it = dd.find(key);
    return (it == dd.end() ? -1 : it->second);
};

cout << f(22) << endl; // should return -1, no 22 key
```
