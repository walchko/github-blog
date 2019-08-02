# C++17 Map

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
