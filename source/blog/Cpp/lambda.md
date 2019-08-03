---
# Lambdas
date: 3 Aug 2019
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
