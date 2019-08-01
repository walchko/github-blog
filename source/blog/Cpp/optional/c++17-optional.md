# C++17 Optional

```cpp
#include <optional>
#include <iostream>

using namespace std;

optional<int> func(){
    return optional<int>();
}

int main(){
    auto i = func();
    cout << i.auto() << endl;
    
    auto ii = func();
    cout << ii.value_or(21) << endl;
    
    return 0;
}
```
