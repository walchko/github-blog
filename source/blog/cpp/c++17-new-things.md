---
title: Some Cool c++17 Things
date: 4 Jun 2019
abstract: Some cool things with C++17
image: "https://i.pinimg.com/564x/1f/62/72/1f62727375a2741af68b0ad470c53645.jpg"
image-height: "300px"
---

## Various New Things

```c++
#include <iostream>
#include <vector>
#include <map>
#include <stdint.h>
#include <optional>
#include <any>

using namespace std;

optional<int> convert(const string& s) {
    try {
        int res = stoi(s);
        return res;
    }
    catch(exception&) {
        return {};
    }
}

int main(){
    // optional return - succes
    int v = convert("123").value_or(0);
    cout << v << endl;
    
    // optional return - fail, uses default
    int v1 = convert("abc").value_or(0);
    cout << v1 << endl;

    // vector with mixed data types
    vector<any> vv { 1, 2.2, false, "hi!" };

    auto& t = vv[1].type();  // What is being held in this any?
    if (t == typeid(double))
        cout << "We have a double" << "\n";
    else
        cout << "We have a problem!" << "\n";

    cout << any_cast<double>(vv[1]) << endl;
    
    try {
        cout << any_cast<int>(vv[1]) << endl;
    } catch(bad_any_cast&) {
        cout << "wrong type" << endl;
    }
    
    // unpack arrays like python
    std::array<int32_t, 6> arr{10, 11, 12, 13, 14, 15};
    auto [i, j, k, l, _dummy1, _dummy2] = arr;
    
    // iterate through vectors
    vector<int> v = {1, 2, 5, 2};
    for (auto const& i: v) cout << i << ' ';
    cout << '\n';
    // prints "1 2 5 2"
    
    // iterate through maps
    map<int,int> m = {{1,2},{3,4}};
    for (const auto& [key, val]: m) cout << key << " " << value << endl;

    return 0;
}
```

## List

- Searching (linear time).
- Inserting, deleting, moving (takes constant time).
- Elements may be ordered.
- Elements may be sorted.
- Elements may be duplicate.

## Set

- Searching (logarithmic in size).
- Insert and delete (logarithimic in general).
- Elements are un-ordered.
- Elements are always sorted from lower to higher.
- Elements are unique.

## Example

If you have a bunch of integers 6, 8, 13, 8, 20, 6 and 50

- set: `{ 6, 8, 13, 20, 50 }`
- list:`{ 6, 8, 13, 8, 20, 6, 50 }`

# References

- [oreilly.com](https://www.oreilly.com/ideas/c++17-upgrades-you-should-be-using-in-your-code)
